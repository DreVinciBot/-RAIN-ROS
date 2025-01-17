#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_listener.h>
#include <iostream>

#include <std_msgs/Float64.h>

using namespace std;


ros::Publisher pub;
ros::Publisher pub_test;
ros::Publisher pose_pub;
ros::Publisher filtered_pub_;


geometry_msgs::Point num;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
// ros::Publisher publisher;

PointCloudT::Ptr cloud (new PointCloudT);

sensor_msgs::PointCloud2 cloud_ros;
sensor_msgs::PointCloud out_pointcloud;

geometry_msgs::PoseArray poses;
geometry_msgs::Point32 point;

tf::TransformListener *tf_listener_;


geometry_msgs::Point product;

std_msgs::Float64 voxel_size;
std_msgs::Float64 xValue;


// Mutex: //
boost::mutex cloud_mutex;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  product = num;
  pub_test.publish(product);

  // voxel_size = product->x;

  std_msgs::Float64 scale, test_variable;
  scale.data = 0.01;



  // test_variable = xValue.data;
  double x = xValue.data * scale.data;
  test_variable.data = x;
  pub.publish(test_variable);
  cloud_ros = *cloud_msg;

  string frame_chosen = "/camera_link";   
  // string frame_chosen = "/camera_depth_frame";  
  // string frame_chosen = "/camera_depth_optical_frame";
  // string frame_chosen = "/base_link";


  tf_listener_->waitForTransform(cloud_msg->header.frame_id,frame_chosen,ros::Time(0), ros::Duration(3.0)); 

  pcl_ros::transformPointCloud (frame_chosen, cloud_ros, cloud_ros, *tf_listener_);


  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(cloud_ros, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (x, x, x);
  sor.filter (cloud_filtered);





  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  

  sensor_msgs::convertPointCloud2ToPointCloud(output, out_pointcloud);
  // cout << out_pointcloud;

  poses.header.stamp = ros::Time::now();
  poses.header.frame_id = frame_chosen;

  for(int i = 0 ; i < out_pointcloud.points.size(); ++i)
  {

    geometry_msgs::Pose pose;
    pose.position.x = out_pointcloud.points[i].x;
    pose.position.y = out_pointcloud.points[i].y;
    pose.position.z = out_pointcloud.points[i].z;
  
    poses.poses.push_back(pose);

  }


  pose_pub.publish(poses);

}

void voxelSizeCallback(const geometry_msgs::Point::ConstPtr& msg)
{

  // double voxel_size;
  num = *msg;
  // voxel_size = msg->x; 
  // std_msgs::Float64 xValue;
  xValue.data = num.x;


}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "RAIN_pointcloud");
  ros::NodeHandle nh;


  tf::TransformListener tf_listener_local;
  tf_listener_ = &tf_listener_local;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);

  ros::Subscriber sub_param = nh.subscribe("/voxelGrid/leafSize", 1, voxelSizeCallback);

  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("/ARFUROS/PointCloud2", 1);
  // filtered_pub_ = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("/ARFUROS/PointCloud2", 1);
  pose_pub = nh.advertise<geometry_msgs::PoseArray> ("/ARFUROS/3DPointArray", 1);

  pub_test = nh.advertise<geometry_msgs::Point>("/talker", 1);

  pub = nh.advertise<std_msgs::Float64>("/Float64", 1);
  // Spin
  ros::spin ();
}
