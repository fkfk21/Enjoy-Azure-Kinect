
#include "image_processor/target_extractor.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto target_extractor = std::make_shared<TargetExtractor>();
    rclcpp::spin(target_extractor);
    rclcpp::shutdown();
    return 0;
}