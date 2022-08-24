#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void pointcloud_callback (const sensor_msgs::PointCloud2& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;

  // Get the field structure of this point cloud
  int pointBytes = cloud_msg.point_step;
  int width = cloud_msg.width;
  int offset_x;
  int offset_y;
  int offset_z;
  int offset_int;
  int steps = 0;
  // ROS_INFO("%i", cloud_msg.data.size());
  for (int f=0; f<cloud_msg.fields.size(); ++f)
  {
    if (cloud_msg.fields[f].name == "x")
      offset_x = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "y")
      offset_y = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "z")
      offset_z = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "intensity")
      offset_int = cloud_msg.fields[f].offset;
  }

  // populate point cloud object
  for (int h=0; h<cloud_msg.height; ++h)
  {
    for (int p=0; p<cloud_msg.width; ++p)
    {
      pcl::PointXYZI newPoint;

      newPoint.x = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + (pointBytes*width*h) + offset_x);
      newPoint.y = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + (pointBytes*width*h) + offset_y);
      newPoint.z = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + (pointBytes*width*h) + offset_z);
      newPoint.intensity = *(unsigned char*)(&cloud_msg.data[0] + (pointBytes*p) + (pointBytes*width*h) + offset_int);

      cloud.points.push_back(newPoint);
      steps = steps + 1;
    }
  }
  //ROS_INFO("%i", steps);
  // Publish the data
  sensor_msgs::PointCloud2 msg{};
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "velodyne";
  msg.header.stamp = ros::Time::now();
  pub.publish(msg);
}
 
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/os_cloud_node/points", 1, pointcloud_callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/points_raw", 1);
  //pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("output", 1);

  // Spin
  ros::spin ();
}
