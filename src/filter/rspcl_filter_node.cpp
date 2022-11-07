#include "rclcpp/rclcpp.hpp"
#include "ros2_rs_pcl/filter/rspcl_filter_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RspclFilterComponent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}