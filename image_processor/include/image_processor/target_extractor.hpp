#ifndef IMAGE_PROCESSOR_INCLUDE_IMAGE_PROCESSOR_TARGET_EXTRACTOR
#define IMAGE_PROCESSOR_INCLUDE_IMAGE_PROCESSOR_TARGET_EXTRACTOR

#include <cmath>
#include <cv_bridge/cv_bridge.h>
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
    : Node("target_extractor"),
      bg_lpf_time_constant_(300), bg_diff_threshold_(100),
      rgb_image_(std::nullopt), depth_image_(std::nullopt),
      rgb_background_image_(std::nullopt), background_image_(std::nullopt),
      depth_camera_info_(nullptr), angle_(std::vector<double>()) {
    // parameterの設定
    this->declare_parameter<int>("bg_lpf_time_constant", 300);
    this->declare_parameter<int>("bg_diff_threshold", 100);
    bg_lpf_time_constant_ = this->get_parameter("bg_lpf_time_constant").as_int();
    bg_diff_threshold_ = this->get_parameter("bg_diff_threshold").as_int();

    // subscriberの設定
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "depth_to_rgb/image_raw", qos,
        std::bind(&TargetExtractor::DepthCallback, this, std::placeholders::_1));

    rgb_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "rgb/image_raw", qos,
        std::bind(&TargetExtractor::RGBCallback, this, std::placeholders::_1));

    depth_camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth_to_rgb/camera_info", qos,
        std::bind(&TargetExtractor::DepthCameraInfoCallback, this, std::placeholders::_1));

    angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "angle", qos, std::bind(&TargetExtractor::AngleCallback, this, std::placeholders::_1));

    // publisherの設定
    result_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("result", qos);
    banished_rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("banished_rgb", qos);

    // serviceの設定
    clear_background_service_ = this->create_service<std_srvs::srv::Empty>(
        "clear_background", std::bind(&TargetExtractor::ClearBackgroundCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
  }

private:
  // メンバ変数
  int bg_lpf_time_constant_;
  int bg_diff_threshold_;
  rclcpp::Time pre_process_depth_time_;
  std::optional<cv::Mat> rgb_image_, depth_image_, rgb_background_image_, background_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr depth_camera_info_;
  std::vector<double> angle_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angle_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr banished_rgb_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_background_service_;


  void RGBCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    rgb_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;

    if (!rgb_background_image_.has_value()) {
      rgb_background_image_ = rgb_image_->clone();
    }
  }

  void DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::toCvCopy(msg, msg->encoding)->image.convertTo(*depth_image_, CV_32F);
    if (!background_image_.has_value()) {
      background_image_ = depth_image_->clone();
      pre_process_depth_time_ = msg->header.stamp;
    }

    // ターゲットの抽出
    ExtractTarget();
  }

  void DepthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    depth_camera_info_ = msg;
  }

  void AngleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) { angle_ = msg->data; }

  void ExtractTarget() {
    if (!depth_image_.has_value() || !background_image_.has_value() || 
        !depth_camera_info_ || angle_.empty())
      return;

    cv::Mat background_diff, background_diff_mask;
    cv::absdiff(background_image_.value(), depth_image_.value(), background_diff);
    cv::threshold(background_diff, background_diff_mask, bg_diff_threshold_, 255,
                  cv::THRESH_BINARY);

    // モルフォロジーのオープニング処理
    cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
    cv::morphologyEx(background_diff_mask, background_diff_mask, cv::MORPH_OPEN, kernel);

    // 輪郭の検出
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(background_diff_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat mask = cv::Mat::zeros(depth_image_->size(), CV_8U);
    for (const auto &angle : angle_) {
      double fx = depth_camera_info_->k[0];
      double cx = depth_camera_info_->k[2];
      double x = -tan(angle) * fx + cx;
      double y = depth_camera_info_->height / 2;

      // (x, y)が含まれる輪郭を抽出 -> maskを作る
      cv::Mat contour_mask = cv::Mat::zeros(depth_image_->size(), CV_8U);
      for (const auto &contour : contours) {
        if (cv::pointPolygonTest(contour, cv::Point(x, y), false) >= 0) {
          cv::drawContours(contour_mask, std::vector<std::vector<cv::Point>>{contour}, -1, 255, -1);
          break;
        }
      }

      cv::Mat contour_depth;
      cv::bitwise_and(depth_image_.value(), depth_image_.value(), contour_depth, contour_mask);

      // Cannyエッジ検出
      cv::Mat edge;
      cv::Canny(contour_depth, edge, 100, 200);  // エッジ取得
      cv::dilate(edge, edge, cv::Mat(), cv::Point(-1, -1), 3);  // エッジを太くする
      cv::bitwise_not(edge, edge);
      cv::bitwise_and(contour_mask, contour_mask, contour_mask, mask=edge);
      cv::findContours(contour_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      for (const auto &contour : contours) {
        if (cv::pointPolygonTest(contour, cv::Point(x, y), false) >= 0) {
          cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, 255, -1);
          break;
        }
      }
    }

    // 結果画像の送信
    auto result_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
    result_msg->header.stamp = pre_process_depth_time_;
    result_msg->header.frame_id = depth_camera_info_->header.frame_id;
    result_publisher_->publish(*result_msg);
  }

  void ClearBackgroundCallback(const std_srvs::srv::Empty::Request::SharedPtr,
                               std_srvs::srv::Empty::Response::SharedPtr) {
    rgb_background_image_ = std::nullopt;
    background_image_ = std::nullopt;
  }

};

#endif /* IMAGE_PROCESSOR_INCLUDE_IMAGE_PROCESSOR_TARGET_EXTRACTOR */
