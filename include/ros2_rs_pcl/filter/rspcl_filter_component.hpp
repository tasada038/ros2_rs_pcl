#ifndef ROS2_RS_PCL__RSPCL_FILTER_COMPONENT_HPP_
#define ROS2_RS_PCL__RSPCL_FILTER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


class RspclFilterComponent : public rclcpp::Node
{
  public:
    RspclFilterComponent();

  private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    double leaf_size_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    size_t count_;
};

#endif // ROS2_RS_PCL__RSPCL_FILTER_COMPONENT_HPP_