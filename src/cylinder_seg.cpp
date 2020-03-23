#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

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
geometry_msgs::Point32 passthrough;
geometry_msgs::Point voxel_size, raduis_limit;

std_msgs::Float64 voxel_dimension, cylinder_radius ,passthrough_z_max;

typedef pcl::PointXYZ PointT;
typedef pcl::PCLPointCloud2 PCL2;

tf::TransformListener *tf_listener_;

sensor_msgs::PointCloud2 cloud_ros;
sensor_msgs::PointCloud2 plane_ros;
sensor_msgs::PointCloud plane_output;
sensor_msgs::PointCloud plane_output_pointxyz;

ros::Publisher pointArray_plane_pub;
ros::Publisher pointArray_cylinder_pub;

ros::Publisher passthrough_pub;
ros::Publisher passthrough_visual_pub;
ros::Publisher voxel_pub;
ros::Publisher radius_pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  cloud_ros = *cloud_msg;

  string frame_chosen = "/camera_depth_frame";
  tf_listener_->waitForTransform(cloud_msg->header.frame_id,frame_chosen,ros::Time(0), ros::Duration(3.0)); 
  pcl_ros::transformPointCloud (frame_chosen, cloud_ros, cloud_ros, *tf_listener_);
  // pcl_ros::transformPointCloud (frame_chosen, raw_data, raw_data, *tf_listener_);


  // All the objects needed
  pcl::PassThrough<PCL2> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  // pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  // pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);


  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
  pcl_conversions::toPCL(cloud_ros, *cloud);


  // Perform filtering voxel group
  pcl::VoxelGrid<PCL2> sor;
  sor.setInputCloud(cloudPtr);
  double x = voxel_dimension.data;
  sor.setLeafSize (x, x, x);
  sor.filter (cloud_filtered);


  sensor_msgs::PointCloud2 voxel_output;
  // pcl_conversions::moveFromPCL(cloud_filtered, voxel_output);
  // pcl::PCLPointCloud2ConstPtr cloudPtr(voxel_output);

  // Build a passthrough filter to remove spurious NaNs
  // pass.setInputCloud (voxel_output);
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("z");


  double z_max = passthrough_z_max.data;
  // pass.setFilterLimits (-1*z_max, z_max);
  pass.setFilterLimits (-1.0, 1.0);
  pass.filter(cloud_filtered);





  pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
  pcl::fromPCLPointCloud2(cloud_filtered, *temp_cloud);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (temp_cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (temp_cloud);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (temp_cloud);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter(*cloud_plane);

  
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);

  // Set the radius limits 
  double r = cylinder_radius.data; 
  seg.setRadiusLimits (0, r);

  // seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    // writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    
    // Send planar points to a message
    ros_rain::PointArray pointArray_cylinder;
    pointArray_cylinder.points.clear();
    pointArray_cylinder.header.stamp = ros::Time::now();
    pointArray_cylinder.header.frame_id = frame_chosen;

    for(int i = 0 ; i < cloud_cylinder->points.size(); ++i)
    {

      geometry_msgs::Point point;

      point.x = cloud_cylinder->points[i].x;
      point.y = cloud_cylinder->points[i].y;
      point.z = cloud_cylinder->points[i].z;

      pointArray_cylinder.points.push_back(point);

    }

    pointArray_cylinder_pub.publish(pointArray_cylinder);
  }

  // Send planar points to a message
  ros_rain::PointArray pointArray_plane;
  pointArray_plane.points.clear();
  pointArray_plane.header.stamp = ros::Time::now();
  pointArray_plane.header.frame_id = frame_chosen;

  for(int i = 0 ; i < cloud_plane->points.size(); ++i)
  {

    geometry_msgs::Point point;

    point.x = cloud_plane->points[i].x;
    point.y = cloud_plane->points[i].y;
    point.z = cloud_plane->points[i].z;

    pointArray_plane.points.push_back(point);

  }

  pointArray_plane_pub.publish(pointArray_plane);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud outpoint_cloud;

  pcl_conversions::moveFromPCL(cloud_filtered, output);
  passthrough_visual_pub.publish(output);

  sensor_msgs::convertPointCloud2ToPointCloud(output, outpoint_cloud);


  std::cerr << "PointCloud after filtering has: " << outpoint_cloud.points.size() << " data points." << std::endl;

}

void voxelSizeCallback(const geometry_msgs::Point::ConstPtr& msg)
{

  voxel_size = *msg;
  voxel_dimension.data = voxel_size.x;
  voxel_pub.publish(voxel_dimension);

}


void radiusLimitsCallback(const geometry_msgs::Point::ConstPtr& msg)
{

  raduis_limit = *msg;
  cylinder_radius.data = raduis_limit.x;
  radius_pub.publish(cylinder_radius);

}

void PassThroughCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
  passthrough = *msg;
  passthrough_z_max.data = passthrough.z;
  passthrough_pub.publish(passthrough_z_max);
}


int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "pointcloud_cylinder");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener_local;
  tf_listener_ = &tf_listener_local;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);

  pointArray_plane_pub = nh.advertise<ros_rain::PointArray>("/RAIN/PointArray_plane", 1);
  pointArray_cylinder_pub = nh.advertise<ros_rain::PointArray>("/RAIN/PointArray_cylinder", 1);


  ros::Subscriber sub_param = nh.subscribe("/voxelGrid/leafSize", 1, voxelSizeCallback);  // voxel_pub = nh.advertise<geometry_msgs::Point32>("RAIN/voxel_size",1)

  ros::Subscriber sub_radius = nh.subscribe("/RAIN/Cylinder/radius", 1, radiusLimitsCallback);  // voxel_pub = nh.advertise<geometry_msgs::Point32>("RAIN/voxel_size",1)


  ros::Subscriber PassThrough_sub = nh.subscribe("/RAIN/passthrough", 1, PassThroughCallback);
  passthrough_pub = nh.advertise<std_msgs::Float32>("/RAIN/PassThrough/z",1);
  passthrough_visual_pub = nh.advertise<sensor_msgs::PointCloud2>("/RAIN/PassThrough/PointCloud2",1);

  voxel_pub = nh.advertise<std_msgs::Float64>("/RAIN/voxel_size",1);
  radius_pub = nh.advertise<std_msgs::Float64>("/RAIN/Cylinder_radius",1);

  ros::spin();

  // // All the objects needed
  // pcl::PCDReader reader;
  // pcl::PassThrough<PointT> pass;
  // pcl::NormalEstimation<PointT, pcl::Normal> ne;
  // pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  // pcl::PCDWriter writer;
  // pcl::ExtractIndices<PointT> extract;
  // pcl::ExtractIndices<pcl::Normal> extract_normals;
  // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // // Datasets
  // pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  // pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  // pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  // pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  // reader.read ("/home/turtlebot/catkin_ws/src/RAIN-ROS/target_image/table_scene_mug_stereo_textured.pcd", *cloud);
  // std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // // Build a passthrough filter to remove spurious NaNs
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0, 1.5);
  // pass.filter (*cloud_filtered);
  // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // // Estimate point normals
  // ne.setSearchMethod (tree);
  // ne.setInputCloud (cloud_filtered);
  // ne.setKSearch (50);
  // ne.compute (*cloud_normals);

  // // Create the segmentation object for the planar model and set all the parameters
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  // seg.setNormalDistanceWeight (0.1);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.03);
  // seg.setInputCloud (cloud_filtered);
  // seg.setInputNormals (cloud_normals);
  // // Obtain the plane inliers and coefficients
  // seg.segment (*inliers_plane, *coefficients_plane);
  // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // // Extract the planar inliers from the input cloud
  // extract.setInputCloud (cloud_filtered);
  // extract.setIndices (inliers_plane);
  // extract.setNegative (false);

  // // Write the planar inliers to disk
  // pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  // extract.filter (*cloud_plane);
  // std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  // writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // // Remove the planar inliers, extract the rest
  // extract.setNegative (true);
  // extract.filter (*cloud_filtered2);
  // extract_normals.setNegative (true);
  // extract_normals.setInputCloud (cloud_normals);
  // extract_normals.setIndices (inliers_plane);
  // extract_normals.filter (*cloud_normals2);

  // // Create the segmentation object for cylinder segmentation and set all the parameters
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_CYLINDER);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setNormalDistanceWeight (0.1);
  // seg.setMaxIterations (10000);
  // seg.setDistanceThreshold (0.05);
  // seg.setRadiusLimits (0, 0.1);
  // seg.setInputCloud (cloud_filtered2);
  // seg.setInputNormals (cloud_normals2);

  // // Obtain the cylinder inliers and coefficients
  // seg.segment (*inliers_cylinder, *coefficients_cylinder);
  // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // // Write the cylinder inliers to disk
  // extract.setInputCloud (cloud_filtered2);
  // extract.setIndices (inliers_cylinder);
  // extract.setNegative (false);
  // pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  // extract.filter (*cloud_cylinder);
  // if (cloud_cylinder->points.empty ()) 
  //   std::cerr << "Can't find the cylindrical component." << std::endl;
  // else
  // {
	 //  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	 //  writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  // }
  // return (0);
}
