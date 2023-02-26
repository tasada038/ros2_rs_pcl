#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "ros2_rs_pcl/clustering/rspcl_clustering_component.hpp"


RspclClusteringComponent::RspclClusteringComponent() : Node("pclsub")
{
  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/pointcloud", 
    10, 
    std::bind(&RspclClusteringComponent::timer_callback, this, std::placeholders::_1)\
  );

  using namespace std::chrono_literals;
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 10);
}


void RspclClusteringComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",cloud_msg->height,cloud_msg->width);

  // define a new container for the data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Voxel Grid: pattern 1
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  voxelGrid.setInputCloud(cloud);
  // set the leaf size (x, y, z)
  voxelGrid.setLeafSize(0.02, 0.02, 0.02);
  // apply the filter to dereferenced cloudVoxel
  voxelGrid.filter(*cloud_filtered);

  // LeafSizeを細かくしすぎると、エラーとなり、止まる
  // [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.

  // SAC Segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
  // Create the segmentation object  
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
  double threshould = 0.01;
  // Optional  
  seg.setOptimizeCoefficients (true);  
  // Mandatory  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);  
  seg.setDistanceThreshold (threshould);  
  seg.setInputCloud (cloud_filtered);  
  seg.segment (*inliers, *coefficients);  

  for (size_t i = 0; i < inliers->indices.size (); ++i) {
    cloud_filtered->points[inliers->indices[i]].r = 255;  
    cloud_filtered->points[inliers->indices[i]].g = 0;  
    cloud_filtered->points[inliers->indices[i]].b = 0;  
  }  

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter (*cloud_filtered);


  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
  // specify euclidean cluster parameters
  ece.setClusterTolerance (0.02); // 2cm
  ece.setMinClusterSize (20);
  ece.setMaxClusterSize (10000);
  ece.setSearchMethod (tree);
  ece.setInputCloud (cloud_filtered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ece.extract (cluster_indices);


  pcl::PCDWriter writer;
  int j = 0;  
  float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
  pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
    {  
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
  cloud_cluster->points[*pit].r = colors[j%6][0];  
  cloud_cluster->points[*pit].g = colors[j%6][1];  
  cloud_cluster->points[*pit].b = colors[j%6][2];  
      }  
      // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
      // std::stringstream ss;  
      // ss << "cloud_cluster_" << j << ".pcd";  
      // writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);  
      j++;  
    }  

  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_cluster, sensor_msg);
  publisher_->publish(sensor_msg);
}