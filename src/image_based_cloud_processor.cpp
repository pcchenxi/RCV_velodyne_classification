#include "pointcloud_image_convertor.h"
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

using namespace std;

Cloud_Image_Convertor   toImage(30, 30, 0.05);
float                   robot_x, robot_y;

void callback_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{    
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud_in, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    copyPointCloud(pcl_cloud, cloud); 

    robot_x = cloud.points[0].x;
    robot_y = cloud.points[0].y;
    
    cout <<"robot x   " << robot_x << "  robot y  "<< robot_y << endl;
    
    //toImage.load_points(cloud, robot_x, robot_y);
  
    cout <<"loaded   " << ros::Time::now() << endl;
}

// void callback_odom_icp(const nav_msgs::Odometry::ConstPtr& odomsg)
// {
//     robot_x = odomsg->pose.pose.position.x;
//     robot_y = odomsg->pose.pose.position.y;
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "image_based_cloud_processor");

    ros::NodeHandle node;
    ros::Rate rate(10.0); 

    ros::Subscriber sub_assenbled = node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1, callback_cloud);

    //ros::Subscriber sub_odom = node.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose", 1, callback_odom);
    // ros::Subscriber sub_odom_icp = node.subscribe<nav_msgs::Odometry >("/icp_odom", 1, callback_odom_icp);

    while (node.ok())
    {

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
