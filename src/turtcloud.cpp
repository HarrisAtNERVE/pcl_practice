#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
//pcl specifics
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing_rgb.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2::ConstPtr & cloud_msg){

    //data container
    //pcl::PointCloud <pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    //pcl::PointCloud <pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
 if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("region_growing_rgb_tutorial.pcd", *cloud) == -1 )
 {
   std::cout << "Cloud reading failed." << std::endl;
   return (-1);
 }
      
    //data processing
    pcl::fromROSMsg(*cloud_msg, *tree);
    sensor_msgs::PointCloud2 output_msg;
    
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (tree);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);

    output_msg_header.frame_id = cloud_msg-> header.frame_id;
    output_msg_header.stamp = cloud_msg-> header.stamp;

    pcl::toROSMsg(*cloud, output_msg);

    //publish data
    pub.publish (output_msg);
}

int
main (int argc, char** argv){

    //init. ROS
    ros::init(argc, argv, "turthouse_node");
    ros::NodeHandle nh;

    //create sub
    ros::Subscriber sub =
      nh.subscribe<sensor_msgs::PointCloud2>
      ("/camera/depth/points", 1, cloud_cb);

    //create pub
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    //maintain link
    ros::spin();
}