#ifndef IMAGE_PROCESSOR_INCLUDE_IMAGE_PROCESSOR_TARGET_EXTRACTOR
#define IMAGE_PROCESSOR_INCLUDE_IMAGE_PROCESSOR_TARGET_EXTRACTOR

#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>

class TargetExtractor : public rclcpp::Node {
public:
  TargetExtractor()
      : Node("target_extractor"), bg_lpf_time_constant_(300), bg_diff_threshold_(100),
        rgb_image_(std::nullopt), depth_image_(std::nullopt), rgb_background_image_(std::nullopt),
        depth_background_image_(std::nullopt), result_image_(std::nullopt),
        depth_camera_info_(nullptr), angle_(std::vector<double>()) {
    // parameterの設定
    this->declare_parameter<int>("bg_lpf_time_constant", 300);
    this->declare_parameter<int>("bg_diff_threshold", 100);
    bg_lpf_time_constant_ = this->get_parameter("bg_lpf_time_constant").as_int();
    bg_diff_threshold_ = this->get_parameter("bg_diff_threshold").as_int();

    // subscriberの設定
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/depth_to_rgb/image_raw", qos,
        std::bind(&TargetExtractor::DepthCallback, this, std::placeholders::_1));

    rgb_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/rgb/image_raw", qos,
        std::bind(&TargetExtractor::RGBCallback, this, std::placeholders::_1));

    depth_camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/depth_to_rgb/camera_info", qos,
        std::bind(&TargetExtractor::DepthCameraInfoCallback, this, std::placeholders::_1));

    rgb_camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/rgb/camera_info", qos,
        std::bind(&TargetExtractor::DepthCameraInfoCallback, this, std::placeholders::_1));

    result_camera_info_publisher_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>("/result/camera_info", qos);

    angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/angle", qos, std::bind(&TargetExtractor::AngleCallback, this, std::placeholders::_1));

    // publisherの設定
    result_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/result/image_raw", qos);
    banished_rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/banished_rgb", qos);

    // serviceの設定
    clear_background_service_ = this->create_service<std_srvs::srv::Empty>(
        "/clear_background", std::bind(&TargetExtractor::ClearBackgroundCallback, this,
                                       std::placeholders::_1, std::placeholders::_2));
  }

private:
  // メンバ変数
  int bg_lpf_time_constant_;
  int bg_diff_threshold_;
  rclcpp::Time pre_process_depth_time_;
  std::optional<cv::Mat> rgb_image_, depth_image_, rgb_background_image_, depth_background_image_,
      result_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr depth_camera_info_;
  std::vector<double> angle_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_camera_info_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angle_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr result_camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr banished_rgb_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_background_service_;

  void RGBCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    rgb_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;

    if (!rgb_background_image_.has_value()) {
      cv::Mat rgb_background_image;
      cv::imwrite(fmt::format("/home/fukuda/rgb_{}.png", msg->header.stamp.sec), *rgb_image_);
      cv::imread(fmt::format("/home/fukuda/rgb_1730683793.png"), cv::IMREAD_COLOR)
          .convertTo(rgb_background_image, CV_8UC3);
      rgb_background_image_ = rgb_background_image;
      // rgb_background_image_ = rgb_image_->clone();
      RCLCPP_INFO(this->get_logger(), "Background RGB image is set.");
    }
  }

  void DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding)->image; // CV_16UC1
    // depth_image_->convertTo(*depth_image_, CV_32F);
    // depth_image_->convertTo(*depth_image_, CV_8U);
    if (!depth_background_image_.has_value()) {
      cv::Mat depth_background_image;
      cv::imwrite(fmt::format("/home/fukuda/depth_{}.png", msg->header.stamp.sec), *depth_image_);
      cv::imread(fmt::format("/home/fukuda/depth_1730683793.png"), cv::IMREAD_GRAYSCALE)
          .convertTo(depth_background_image, CV_16UC1);
      depth_background_image_ = depth_background_image;
      // depth_background_image_ = depth_image_->clone();
      pre_process_depth_time_ = msg->header.stamp;
      RCLCPP_INFO(this->get_logger(), "Background Depth image is set.");
    }

    // ターゲットの抽出
    ExtractTarget();
  }

  void DepthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    depth_camera_info_ = msg;
    result_camera_info_publisher_->publish(*msg);
  }

  void AngleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) { angle_ = msg->data; }

  void ExtractTarget() {
    if (!depth_image_.has_value() || !depth_background_image_.has_value() ||
        !rgb_image_.has_value() || !depth_camera_info_) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Extracting target for %d angles", angle_.size());

    cv::Mat scaled_depth_image; // image from CV_16UC1 to CV_8U
    // cv::normalize(depth_image_.value(), scaled_depth_image, 0, 255, cv::NORM_MINMAX, CV_8U);
    const uint16_t depth_max = 50000;
    cv::threshold(*depth_image_, *depth_image_, depth_max, 0, cv::THRESH_TOZERO_INV);
    depth_image_->convertTo(scaled_depth_image, CV_8U);

    // スケーリング範囲 (2000mmから6000mmを0から255にスケーリング)
    // double minDepth = 200.0;
    // double maxDepth = 6000.0;
    // double scale = 255.0 / (maxDepth - minDepth);

    // // スケーリング変換
    // depth_image_->convertTo(scaled_depth_image, CV_8U, scale, -minDepth * scale);

    // // Cannyエッジ検出
    // cv::Mat edges;
    // cv::Canny(scaled_depth_image, edges, 50, 150);

    // // エッジ部分を削除
    // // 16ビットの元画像のエッジに対応するピクセルを0に設定
    // cv::Mat img_no_edges = scaled_depth_image.clone();
    // img_no_edges.setTo(0, edges); // エッジ部分が黒（0）になります

    // cv::imshow("no_edges", img_no_edges);

    depth_image_ = scaled_depth_image;
    // cv::normalize(*depth_background_image_, *depth_background_image_, 0, 255, cv::NORM_MINMAX,
    //               CV_8U);
    depth_background_image_->convertTo(depth_background_image_.value(), CV_8U);

    cv::Mat background_diff, background_diff_mask;
    cv::absdiff(depth_background_image_.value(), depth_image_.value(), background_diff);

    // cv::imshow("depth", depth_image_.value());
    // cv::imshow("diff", background_diff);

    cv::threshold(background_diff, background_diff_mask, bg_diff_threshold_, 255,
                  cv::THRESH_BINARY);

    cv::imshow("diff_mask", background_diff);

    // モルフォロジーのオープニング処理
    cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
    cv::morphologyEx(background_diff_mask, background_diff_mask, cv::MORPH_OPEN, kernel);

    // 輪郭の検出
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(background_diff_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat mask = cv::Mat::zeros(depth_image_->size(), CV_8U);
    if (!angle_.empty()) {
      for (const auto &angle : angle_) {
        double fx = depth_camera_info_->k[0];
        double cx = depth_camera_info_->k[2];
        double x = -tan(angle) * fx + cx;
        double y = depth_camera_info_->height * 3.0 / 4.0;

        for (const auto &contour : contours) {
          if (cv::pointPolygonTest(contour, cv::Point(x, y), false) >= 0) {
            cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, 255, -1);
            break;
          }
        }
        y = depth_camera_info_->height * 2.0 / 4.0;
        for (const auto &contour : contours) {
          if (cv::pointPolygonTest(contour, cv::Point(x, y), false) >= 0) {
            cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, 255, -1);
            break;
          }
        }
      }
    }
    // maskの膨張処理
    kernel = cv::Mat::ones(10, 10, CV_8U);
    cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel);


    // RGB画像のマスク処理
    cv::Mat rgb_bg, rgb_fg, reversed_mask;
    cv::bitwise_and(*rgb_background_image_, *rgb_background_image_, rgb_bg, mask);
    cv::bitwise_not(mask, reversed_mask);
    cv::bitwise_and(*rgb_image_, *rgb_image_, rgb_fg, reversed_mask);

    cv::add(rgb_bg, rgb_fg, *result_image_);

    if (true) { // debug
      // cv::imshow("scaled_depth", *depth_background_image_);
      // cv::imshow("diff_mask", background_diff_mask);
      cv::imshow("mask", mask);
      // cv::imshow("result", *result_image_);
      cv::waitKey(1);
    }

    // 結果画像の送信
    // auto result_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();

    cv::flip(*result_image_, *result_image_, 1);
    auto result_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *result_image_).toImageMsg();
    result_msg->header.stamp = pre_process_depth_time_;
    result_msg->header.frame_id = depth_camera_info_->header.frame_id;
    result_publisher_->publish(*result_msg);
  }

  void ClearBackgroundCallback(const std_srvs::srv::Empty::Request::SharedPtr,
                               std_srvs::srv::Empty::Response::SharedPtr) {
    rgb_background_image_ = rgb_image_->clone();
    depth_background_image_ = depth_image_->clone();
    cv::imwrite(fmt::format("/home/fukuda/depth_{}.png",
                            static_cast<int>(pre_process_depth_time_.seconds())),
                depth_image_.value());
    cv::imwrite(fmt::format("/home/fukuda/rgb_{}.png",
                            static_cast<int>(pre_process_depth_time_.seconds())),
                rgb_image_.value());
  }
};

#endif /* IMAGE_PROCESSOR_INCLUDE_IMAGE_PROCESSOR_TARGET_EXTRACTOR */
