#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>



const std::string TOPIC_NAME = "/rgb/image_raw";


class ImageCompressor : public rclcpp::Node {
public:
    ImageCompressor() : Node("image_compressor") {
        // パラメータを宣言
        this->declare_parameter<std::string>("input_topic", TOPIC_NAME);
        std::string input_topic = this->get_parameter("input_topic").as_string();
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", input_topic.c_str());

        // サブスクリプションとパブリッシャーの作成
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10, std::bind(&ImageCompressor::listener_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            input_topic + "/compressed", 10);
    }

private:
    void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // 画像データをOpenCVフォーマットに変換
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // 画像をJPEG形式で圧縮
        std::vector<uchar> compressed_data;
        if (!cv::imencode(".jpg", cv_ptr->image, compressed_data)) {
            RCLCPP_WARN(this->get_logger(), "Image compression failed.");
            return;
        }

        // CompressedImageメッセージの作成
        auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_msg->header.stamp = this->get_clock()->now();
        compressed_msg->format = "jpeg";
        compressed_msg->data.assign(compressed_data.begin(), compressed_data.end());

        // 圧縮画像をパブリッシュ
        publisher_->publish(*compressed_msg);
        // RCLCPP_INFO(this->get_logger(), "Published compressed image.");
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto image_compressor = std::make_shared<ImageCompressor>();
    rclcpp::spin(image_compressor);
    rclcpp::shutdown();
    return 0;
}