#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;


class PclSub : public rclcpp::Node
{
  public:
    PclSub(): Node("pclsub")
    {
      sub_novel = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/pointcloud", 
        10, 
        std::bind(&PclSub::topic_callback, this, std::placeholders::_1)\
      );

      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 10);
      publisher_m = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "output_marker", 10);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_novel;

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_msg, *cloud);
      RCLCPP_INFO(this->get_logger(), "cloud_size(%d)",cloud->points.size());
      RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",cloud_msg->height,cloud_msg->width);

      // define a new container for the data
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

      // Voxel Grid: pattern 1
      pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
      voxelGrid.setInputCloud(cloud);
      // set the leaf size (x, y, z)
      voxelGrid.setLeafSize(0.4, 0.4, 0.4);
      // apply the filter to dereferenced cloudVoxel
      voxelGrid.filter(*cloud_filtered);

      // Convert PointCloud2 to Point32 vector
      std::vector<geometry_msgs::msg::Point32> geometry_points;
      geometry_msgs::msg::Point32 p;
      int cloud_len = cloud_filtered->points.size();
      RCLCPP_INFO(this->get_logger(), "cloud_size(%d)",cloud_len);

      for (int i=0; i< cloud_len; i++){
        p.x = cloud_filtered->points[i].x;
        p.y = cloud_filtered->points[i].y;
        p.z = cloud_filtered->points[i].z;
        geometry_points.push_back(p);
      }

      RCLCPP_INFO(this->get_logger(), "geometry_x: %f",geometry_points[1].x);
      RCLCPP_INFO(this->get_logger(), "geometry_y: %f",geometry_points[1].y);
      RCLCPP_INFO(this->get_logger(), "geometry_z: %f",geometry_points[1].z);
      // publisher_m->publish(p);

      // http://wiki.ros.org/rviz/DisplayTypes/Marker
      //https://github.com/garaemon/rviz_collada_marker/blob/master/marker_mesh_sample.py

      visualization_msgs::msg::MarkerArray markers;
      geometry_msgs::msg::Point start_line;
      geometry_msgs::msg::Point goal_line;
      int id = 0;
      for (int i=1; i< cloud_len; i++)
      {
        // Create a marker for the node
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_color_optical_frame";
        marker.header.stamp = this->now();
        marker.ns = "gng_map";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = geometry_points[i].x;
        marker.pose.position.y = geometry_points[i].y;
        marker.pose.position.z = geometry_points[i].z;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0.0);

        // ARROW_r
        visualization_msgs::msg::Marker arrow_r;
        arrow_r.header.frame_id = "camera_color_optical_frame";
        arrow_r.header.stamp = this->now();
        arrow_r.ns = "gng_map";
        arrow_r.id = id++;
        arrow_r.type = visualization_msgs::msg::Marker::ARROW;
        arrow_r.action = visualization_msgs::msg::Marker::ADD;
        arrow_r.pose.position.x = geometry_points[i].x;
        arrow_r.pose.position.y = geometry_points[i].y;
        arrow_r.pose.position.z = geometry_points[i].z;
        arrow_r.pose.orientation.x = 0.0;
        arrow_r.pose.orientation.y = 0.0;
        arrow_r.pose.orientation.z = 0.0;
        arrow_r.pose.orientation.w = 1.0;
        arrow_r.scale.x = 0.1;
        arrow_r.scale.y = arrow_r.scale.z = 0.01;
        arrow_r.color.r = 1.0;
        arrow_r.color.g = 0.0;
        arrow_r.color.b = 0.0;
        arrow_r.color.a = 1.0;
        arrow_r.lifetime = rclcpp::Duration(0.0);

        // ARROW_g
        visualization_msgs::msg::Marker arrow_g;
        arrow_g.header.frame_id = "camera_color_optical_frame";
        arrow_g.header.stamp = this->now();
        arrow_g.ns = "gng_map";
        arrow_g.id = id++;
        arrow_g.type = visualization_msgs::msg::Marker::ARROW;
        arrow_g.action = visualization_msgs::msg::Marker::ADD;
        arrow_g.pose.position.x = geometry_points[i].x;
        arrow_g.pose.position.y = geometry_points[i].y;
        arrow_g.pose.position.z = geometry_points[i].z;
        arrow_g.pose.orientation.x = 0.0;
        arrow_g.pose.orientation.y = -0.707;
        arrow_g.pose.orientation.z = 0.0;
        arrow_g.pose.orientation.w = 0.707;
        arrow_g.scale.x = 0.1;
        arrow_g.scale.y = arrow_g.scale.z = 0.01;
        arrow_g.color.r = 0.0;
        arrow_g.color.g = 1.0;
        arrow_g.color.b = 0.0;
        arrow_g.color.a = 1.0;
        arrow_g.lifetime = rclcpp::Duration(0.0);

        // TEXT
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "camera_color_optical_frame";
        text.header.stamp = this->now();
        text.ns = "gng_map";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = geometry_points[i].x+0.02;
        text.pose.position.y = geometry_points[i].y+0.02;
        text.pose.position.z = geometry_points[i].z-0.02;
        text.scale.z = 0.02;
        text.text = std::to_string(text.pose.position.x) + "," \
        + std::to_string(text.pose.position.y) + "," \
        + std::to_string(text.pose.position.z);
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        text.lifetime = rclcpp::Duration(0.0);

        // LINE STRIP
        visualization_msgs::msg::Marker connection;
        connection.header.frame_id = "camera_color_optical_frame";
        connection.header.stamp = this->now();
        connection.ns = "line_strip";
        connection.id = id++;
        connection.type = visualization_msgs::msg::Marker::LINE_STRIP;
        connection.action = visualization_msgs::msg::Marker::ADD;
        connection.scale.x = 0.005;
        connection.color.r = 0.0;
        connection.color.g = 1.0;
        connection.color.b = 0.0;
        connection.color.a = 1.0;
        connection.lifetime = rclcpp::Duration(0.0);
        start_line.x = geometry_points[i].x;
        start_line.y = geometry_points[i].y;
        start_line.z = geometry_points[i].z;
        goal_line.x = geometry_points[i-1].x;
        goal_line.y = geometry_points[i-1].y;
        goal_line.z = geometry_points[i-1].z;
        connection.points.push_back(start_line);
        connection.points.push_back(goal_line);

        markers.markers.push_back(marker);
        markers.markers.push_back(arrow_r);
        markers.markers.push_back(arrow_g);
        markers.markers.push_back(text); 
        // markers.markers.push_back(connection);  
      }

      publisher_m->publish(markers);

      sensor_msgs::msg::PointCloud2 sensor_msg;
      pcl::toROSMsg(*cloud_filtered, sensor_msg);
      publisher_->publish(sensor_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_m;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PclSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}