#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

#include "image_processor/image_compressor.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    std::vector<std::shared_ptr<rclcpp::Node>> nodes = {
        std::make_shared<ImageCompressor>("/rgb/image_raw"),
        std::make_shared<ImageCompressor>("/depth_to_rgb/image_raw"),
        std::make_shared<ImageCompressor>("/depth/image_raw"),
    };

    for (const auto& node : nodes) {
        exec.add_node(node);
    }
    exec.spin();
    rclcpp::shutdown();
    return 0;
}