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

#include "ros2_rs_pcl/filter/rspcl_filter_component.hpp"


RspclFilterComponent::RspclFilterComponent() : Node("pclsub")
{
  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/pointcloud", 
    10, 
    std::bind(&RspclFilterComponent::timer_callback, this, std::placeholders::_1)\
  );

  using namespace std::chrono_literals;
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 10);
}


void RspclFilterComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",cloud_msg->height,cloud_msg->width);

  // define a new container for the data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // // PassThrough Filter
  // pcl::PassThrough<pcl::PointXYZRGB> pass;
  // pass.setInputCloud(cloud);
  // pass.setFilterFieldName("x");  // x axis
  // // extract point cloud between 1.0 and 3.0 m
  // pass.setFilterLimits(1.0,3.0);
  // // pass.setFilterLimitsNegative (true);   // extract range reverse
  // pass.filter(*cloud_filtered);

  // // Approximate Voxel Grid
  // pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> avg;
  // avg.setInputCloud(cloud);
  // avg.setLeafSize(0.2f, 0.2f, 0.2f);
  // // avg.setDownsampleAllData(true);
  // avg.filter(*cloud_filtered);

  // Voxel Grid: pattern 1
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  voxelGrid.setInputCloud(cloud);
  leaf_size_ = 0.1;
  // set the leaf size (x, y, z)
  voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // apply the filter to dereferenced cloudVoxel
  voxelGrid.filter(*cloud_filtered);

  // // Voxel Grid: pattern 2
  // pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
  // pcl::toPCLPointCloud2(*cloud, *cloud_blob);
  // pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  // vg.setInputCloud(cloud_blob);
  // leaf_size_ = 0.1;
  // vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // vg.filter(*cloud_filtered_blob);
  // pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  // // Statistical Outlier Removal
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  // sor.setInputCloud(cloud);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(0.1);
  // sor.setNegative(false);
  // sor.filter (*cloud_filtered);

  // // Radius Outlier Removal
  // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  // outrem.setInputCloud(cloud);
  // outrem.setRadiusSearch(0.1);
  // outrem.setMinNeighborsInRadius(2);
  // outrem.setKeepOrganized(true);
  // outrem.filter(*cloud_filtered);

  // // Conditional Removal
  // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, 0.0)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, 3.0)));
  // pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  // condrem.setCondition(range_cond);
  // condrem.setInputCloud(cloud);
  // // condrem.setKeepOrganized(true);
  // condrem.filter(*cloud_filtered);
  // // vector<int> Idx;
  // // pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, Idx);


  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_filtered, sensor_msg);
  publisher_->publish(sensor_msg);
}