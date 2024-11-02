#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

#include "image_processor/image_compressor.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto image_compressor = std::make_shared<ImageCompressor>();
    rclcpp::spin(image_compressor);
    rclcpp::shutdown();
    return 0;
}