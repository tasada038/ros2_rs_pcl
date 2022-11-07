#include "rclcpp/rclcpp.hpp"
#include "ros2_rs_pcl/clustering/rspcl_clustering_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RspclClusteringComponent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}