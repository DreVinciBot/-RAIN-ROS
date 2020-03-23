#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf/transform_listener.h>
#include <iostream>

// Custom message types
#include <ros_rain/PointArray.h>

using namespace std;

ros::Publisher pointArray_raw_pub;
ros::Publisher pointArray_voxel_pub;

tf::TransformListener *tf_listener_;

sensor_msgs::PointCloud2 cloud_ros;
sensor_msgs::PointCloud2 raw_data;
sensor_msgs::PointCloud out_raw_data;
sensor_msgs::PointCloud out_pointcloud;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  raw_data = *cloud_msg;
  cloud_ros = *cloud_msg;

  string frame_chosen = "/camera_depth_frame";
  tf_listener_->waitForTransform(cloud_msg->header.frame_id,frame_chosen,ros::Time(0), ros::Duration(3.0)); 
  pcl_ros::transformPointCloud (frame_chosen, cloud_ros, cloud_ros, *tf_listener_);
  // pcl_ros::transformPointCloud (frame_chosen, raw_data, raw_data, *tf_listener_);


  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(cloud_ros, *cloud);


  // pass through filter
  // pcl::PassThrough


  // Perform the actual filtering voxel group
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  double x = 0.01;
  sor.setLeafSize (x, x, x);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;


  pcl_conversions::moveFromPCL(cloud_filtered, output);
  // pcl_conversions::moveFromPCL(raw_data, output);
  

  sensor_msgs::convertPointCloud2ToPointCloud(output, out_pointcloud);
  sensor_msgs::convertPointCloud2ToPointCloud(raw_data, out_raw_data);

  ros_rain::PointArray pointArray_raw;
  ros_rain::PointArray pointArray_voxel;
 
  pointArray_raw.points.clear();

  pointArray_raw.header.stamp = ros::Time::now();
  pointArray_raw.header.frame_id = frame_chosen;

  for(int i = 0 ; i < out_raw_data.points.size(); ++i)
  {

    geometry_msgs::Point point;

  	point.x = out_raw_data.points[i].x;
    point.y = out_raw_data.points[i].y;
    point.z = out_raw_data.points[i].z;

    pointArray_raw.points.push_back(point);

  }

  for(int i = 0 ; i < out_pointcloud.points.size(); ++i)
  {

    geometry_msgs::Point point;

    point.x = out_pointcloud.points[i].x;
    point.y = out_pointcloud.points[i].y;
    point.z = out_pointcloud.points[i].z;

    pointArray_voxel.points.push_back(point);

  }

  pointArray_raw_pub.publish(pointArray_raw);
  pointArray_voxel_pub.publish(pointArray_voxel);

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_rain");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener_local;
  tf_listener_ = &tf_listener_local;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
 
  pointArray_voxel_pub = nh.advertise<ros_rain::PointArray> ("/RAIN/PointArray", 1);
  pointArray_raw_pub = nh.advertise<ros_rain::PointArray>("/RAIN/PointArray_raw", 1);

  ros::spin ();
}