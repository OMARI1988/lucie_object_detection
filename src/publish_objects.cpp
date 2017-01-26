#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
std::stringstream convert;
std::string folder1;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "map";
  msg->height = 1;
  msg->width = 0;

  for( int a = 0; a < 23; a = a + 1 ) {
    //####################################################################################    read pc
    std::ostringstream convert;   // stream used for the conversion
    convert << a;
		// folder1 = "/home/omari/interior_cloud_"+convert.str()+".pcd";
    // folder1 = "/home/omari/complete_cloud_"+convert.str()+".pcd";
    folder1 = "/home/omari/Datasets/metarooms/clusters/cluster_"+convert.str()+".pcd";
    pcl::io::loadPCDFile (folder1.c_str(), *cloud);
    for (int pit = 0; pit < cloud->size() ; pit++)
    {
      msg->points.push_back (cloud->points[pit]); //*
      msg->width += 1;
    }
    // msg->points.push_back (pcl::PointXYZRGB(1.0, 2.0, 3.0));
  }

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    // msg->header.stamp = ros::Time::now().toNSec();
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
