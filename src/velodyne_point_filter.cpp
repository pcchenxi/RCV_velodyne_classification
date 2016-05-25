#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/impl/transforms.hpp"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/ModelCoefficients.h>​


#include <nav_msgs/OccupancyGrid.h>

#include "pointcloud_image_convertor.h"



using namespace std;


ros::Publisher  pub_flat, pub_obs, pub_allclouds, pub_costmap;
tf::TransformListener* tfListener = NULL;
Cloud_Image_Convertor   toImage(30, 30, 0.05);

float       robot_x = 0, robot_y = 0;
bool        odom_recived = true;

pcl::PointCloud<pcl::PointXYZRGB>  cloud_all;

bool map_init = false;
bool map_ready = false;
nav_msgs::OccupancyGrid g_map;

void callback_odom_icp(const nav_msgs::Odometry::ConstPtr& odomsg)
{
    robot_x = odomsg->pose.pose.position.x;
    robot_y = odomsg->pose.pose.position.y;
    odom_recived = true;
}


void callback_odom (geometry_msgs::PoseStamped data)
{
	//double yaw,pitch,roll,tmp;
	//tf::Quaternion q;
	//tf::quaternionMsgToTF(data.pose.orientation, q);
	//tf::Matrix3x3(q).getEulerYPR(yaw, tmp, tmp);

	robot_x = data.pose.position.x;
	robot_y = data.pose.position.y;

	cout << "robot z: " << robot_x << "  robot_y: " << robot_y << endl;

	odom_recived = true;
}

pcl::PointCloud<pcl::Normal>::Ptr calculateSurfaceNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point,
                                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr search_point,
                                                         float searchRadius )
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (input_point);
    ne.setSearchSurface(search_point);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setRadiusSearch (searchRadius);
    ne.setViewPoint (0, 0, 0.55);
    ne.compute (*cloud_normals);

    return cloud_normals;
}

pcl::PointCloud<pcl::PointXYZRGB> cloud_filter(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  input_cloud       (new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_passthrough (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_voxel       (new pcl::PointCloud<pcl::PointXYZRGB>);

    //cout << "before filter  " << input_cloud->points.size() << endl;

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (input_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-2.0, 4.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_passthrough);
    //cout << "after z filter  " << cloud_passthrough->points.size() << endl;


    pass.setInputCloud (cloud_passthrough);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (robot_x-15.0, robot_x + 15.0);
    pass.filter (*cloud_passthrough);
    //cout << "after x filter  " << cloud_passthrough->points.size() << endl;


    pass.setInputCloud (cloud_passthrough);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (robot_y - 15.0, robot_y + 15.0);
    pass.filter (*cloud_passthrough);
    //cout << "after y filter  " << cloud_passthrough->points.size() << endl;

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_passthrough);
    sor.setLeafSize (0.03f, 0.03f, 0.03f);
    sor.filter (*cloud_voxel);
    cout << "after voxel filter  " << cloud_voxel->points.size() << endl;


    // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    // pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
    // pcl::PointCloud<pcl::PointNormal> mls_points;

    // mls.setComputeNormals (false);
    // // Set parameters
    // mls.setInputCloud (cloud_voxel);
    // mls.setPolynomialFit (false);
    // mls.setSearchMethod (tree);
    // mls.setSearchRadius (0.05);
    // mls.process (mls_points);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // mls_cloud->resize(mls_points.size());

    // for (size_t i = 0; i < mls_points.points.size(); ++i)
    // {
    //     mls_cloud->points[i].x=mls_points.points[i].x; //error
    //     mls_cloud->points[i].y=mls_points.points[i].y; //error
    //     mls_cloud->points[i].z=mls_points.points[i].z; //error
    // }
    // mls_cloud->header = cloud_voxel->header;


      // Create the filtering object
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    // sor.setInputCloud (cloud_passthrough);
    // sor.setMeanK (5);
    // sor.setStddevMulThresh (1.0);
    // sor.filter (*cloud_passthrough);

    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // outrem.setInputCloud(cloud_passthrough);
    // outrem.setRadiusSearch(0.1);
    // outrem.setMinNeighborsInRadius (2);
    // outrem.filter (*cloud_passthrough);

    cout <<ros::Time::now() << "   finish filtering"<<endl;
    return *cloud_voxel;
}


void publish(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> cloud, int type = 2)
{
    sensor_msgs::PointCloud2 pointlcoud2;
    pcl::toROSMsg(cloud, pointlcoud2);
    
    if(type == 2)
    {
        
        pub.publish(pointlcoud2);
        cout << "2" << endl;
    }
    else
    {
        sensor_msgs::PointCloud pointlcoud;      
        sensor_msgs::convertPointCloud2ToPointCloud(pointlcoud2, pointlcoud);
        pointlcoud.header = pointlcoud2.header;
        pointlcoud.header.frame_id = "base_link";
        pub.publish(pointlcoud);
    }
    
}


void callback_laser(const sensor_msgs::PointCloudConstPtr &cloud_in)
{
    cout << "laser recieved" << endl;
    sensor_msgs::PointCloud2 cloud_out;
    sensor_msgs::convertPointCloudToPointCloud2(*cloud_in, cloud_out);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_out, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    copyPointCloud(pcl_cloud, cloud);

    cloud_all = cloud_filter(cloud);

    pcl::PointCloud<pcl::Normal>::Ptr       normal      (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_prt   (new pcl::PointCloud<pcl::PointXYZRGB>(cloud_all));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  all_prt     (new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

    normal = calculateSurfaceNormal(cloud_prt, all_prt, 0.15);
    cout << "..... finished                  " << ros::Time::now() << endl;



    cout << "start obstacle detect ...... " << ros::Time::now() << endl;
    cloud_all = toImage.obstaclePointsDetection(cloud_all, robot_x, robot_y, *normal);
    //cloud_all = toImage.obstaclePointsDetection(cloud_all, robot_x, robot_y);
    //publish(pub_allclouds, cloud_all);
    publish(pub_allclouds, toImage.m_all_points);
    cout << "..... finished obstacle detect  " << ros::Time::now() << endl;

    cloud_all = toImage.resampleCloud(cloud_all, robot_x, robot_y);


    //////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////   path planner  /////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    // nav_msgs::Path path = plath_planner.plan(toImage.m_costmap_char, toImage.m_image_rows, toImage.m_image_cols);


    ros::Time time_st = ros::Time::now ();

    cloud_all.header.stamp = time_st.toNSec()/1e3;
    toImage.m_costmap_points.header.stamp = time_st.toNSec()/1e3;
    
    publish(pub_allclouds, cloud_all);
    publish(pub_costmap, toImage.m_costmap_points, 1);


    cout <<"pbulished   "<< cloud_all.points.size() << "  " << ros::Time::now() << endl;

}


void callback_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    // if(!odom_recived)
    //     return;

    // sensor_msgs::PointCloud2 cloud_map;

    // tf::StampedTransform velodyne_to_map;

    // tfListener->waitForTransform("/map", "/velodyne",ros::Time::now(), ros::Duration(3.0));
    // tfListener->lookupTransform("/map", "/velodyne", ros::Time(0), velodyne_to_map);

    // Eigen::Matrix4f eigen_transform;
    // pcl_ros::transformAsMatrix (velodyne_to_map, eigen_transform);
    // pcl_ros::transformPointCloud (eigen_transform, *cloud_in, cloud_map);

    // cloud_map.header.frame_id = "map";

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud_in, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    copyPointCloud(pcl_cloud, cloud);


    cloud_all = cloud_filter(cloud);


    cout << "caldulate normal ........       " << ros::Time::now() << endl;
    pcl::PointCloud<pcl::Normal>::Ptr       normal      (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_prt   (new pcl::PointCloud<pcl::PointXYZRGB>(cloud_all));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  all_prt   (new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

    normal = calculateSurfaceNormal(cloud_prt, all_prt, 0.10);
    cout << "..... finished                  " << ros::Time::now() << endl;



    cout << "start obstacle detect ...... " << ros::Time::now() << endl;
    cloud_all = toImage.obstaclePointsDetection(cloud_all, robot_x, robot_y, *normal);
    //cloud_all = toImage.obstaclePointsDetection(cloud_all, robot_x, robot_y);
    cout << "..... finished obstacle detect  " << ros::Time::now() << endl;

    //generate_costmap(cloud_all);


    cloud_all.points[0].x = robot_x;
    cloud_all.points[0].y = robot_y;

    publish(pub_allclouds, cloud_all);


    cout <<"pbulished   "<< cloud_all.points.size() << "  " << ros::Time::now() << endl;
}

void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    cout << "goal recieved" << endl;

    //costmap_2d::Costmap2DROS* planner_costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", *tfListener);
    //nav_msgs::OccupancyGridPtr pOccMapEmp(new nav_msgs::OccupancyGrid());

//   nav_msgs::OccupancyGrid og;
//   og.header.frame_id = "/map";
//   og.header.stamp = ros::Time::now();
//   og.info.resolution = 0.1;
//   og.info.width = 200;
//   og.info.height = 200;
//   og.data.resize(200 * 200);

//     for(int nX = 0; nX < og.info.width; nX++)
//     {
//         for(int nY = 0; nY < og.info.height; nY++)
//         {
//             int nIndex = nX + nY * og.info.width;
//             // if(pOccMapEmp->data[nIndex] > nZeroThreshold)
//             // {
//                 // pOccMapEmp->data[nIndex] = _nFromColorRange + pOccMapEmp->data[nIndex] * nColorSpan / 100.0;
//             // }
//             // else
//             // {
//                 og.data[nIndex] = 0;
//             // }
//         }
//     }

    // if(map_ready)
    //     pubConfMap.publish(g_map);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_point_filter");

    ros::NodeHandle node;
    ros::Rate rate(10.0);

    //ros::Subscriber sub_assenbled = node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, callback_velodyne);
    //ros::Subscriber sub_assenbled = node.subscribe<sensor_msgs::PointCloud2>("/assembled_cloud", 1, callback_velodyne);
    ros::Subscriber sub_assenbled = node.subscribe<sensor_msgs::PointCloud>("/assembled_laser", 1, callback_laser);
    pub_allclouds = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1);

    ros::Subscriber sub_odom = node.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose", 1, callback_odom);
    ros::Subscriber sub_odom_icp = node.subscribe<nav_msgs::Odometry >("/icp_odom", 1, callback_odom_icp);

    ros::Subscriber goal_sub_ = node.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCB);

    ///////////////////////////////////
    pub_costmap = node.advertise<sensor_msgs::PointCloud>("/costmap_cloud",1);

    tfListener = new (tf::TransformListener);

    while (node.ok())
    {

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
