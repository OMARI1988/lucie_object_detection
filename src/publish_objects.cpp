#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
std::stringstream convert;
std::string folder1;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub1 = nh.advertise<PointCloud> ("original", 1);
  ros::Publisher pub2 = nh.advertise<PointCloud> ("objects", 1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  PointCloud::Ptr msg2 (new PointCloud);
  msg2->header.frame_id = "map";
  msg2->height = 1;
  msg2->width = 0;

  PointCloud::Ptr msg1 (new PointCloud);
  msg1->header.frame_id = "map";
  msg1->height = 1;
  msg1->width = 0;

  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  float x= -4.7;
  float y= -17.0701;
  float z=0;
  trans.translation ().matrix () = Eigen::Vector3f (x,y,z);

  //####################################################################################    read original scene

  for( int a = 0; a < 2; a = a + 1 ) {
    std::ostringstream convert2;   // stream used for the conversion
    convert2 << a;
    folder1 = "/home/omari/Datasets/metarooms/clusters/original"+convert2.str()+".pcd";
    pcl::io::loadPCDFile (folder1.c_str(), *cloud);
    pcl::transformPointCloud<pcl::PointXYZRGB> (*cloud, *cloud, trans);
    for (int pit = 0; pit < cloud->size() ; pit++)
    {
      msg1->points.push_back (cloud->points[pit]); //*
      msg1->width += 1;
    }
  }

  for( int a = 0; a < 42; a = a + 1 ) {
    //####################################################################################    read pc
    std::ostringstream convert;   // stream used for the conversion
    convert << a;
    folder1 = "/home/omari/Datasets/metarooms/clusters/cluster_"+convert.str()+".pcd";
    pcl::io::loadPCDFile (folder1.c_str(), *cloud);
    pcl::transformPointCloud<pcl::PointXYZRGB> (*cloud, *cloud, trans);
    for (int pit = 0; pit < cloud->size() ; pit++)
    {
      msg2->points.push_back (cloud->points[pit]); //*
      msg2->width += 1;
    }
    // msg->points.push_back (pcl::PointXYZRGB(1.0, 2.0, 3.0));
  }

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    // msg->header.stamp = ros::Time::now().toNSec();
    pub1.publish (msg1);
    pub2.publish (msg2);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
