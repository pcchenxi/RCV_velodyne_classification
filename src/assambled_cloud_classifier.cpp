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

#include "pointcloud_image_convertor.h"

using namespace std;


ros::Publisher  pub_flat, pub_obs, pub_allclouds, pub_costmap, pub_stoppose;
tf::TransformListener* tfListener = NULL;
//Cloud_Image_Convertor   toImage(60, 60, 0.05);

pcl::PointCloud<pcl::PointXYZRGB>  cloud_all, laser_cloud_temp, cloud_assambled, cloud_clean;
Cloud_Image_Convertor   toImage(30, 30, 0.1);

float       robot_x = 0, robot_y = 0;
bool        odom_recived = true;
bool        laser_cloud_ready = false;

int         datain_count = 0;


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
    pass.setFilterLimits (-2.0, 3.0);
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

//        tf::StampedTransform base_to_map;
//
//        tfListener->waitForTransform("/map", "/base_link",ros::Time::now(), ros::Duration(3.0));
//        tfListener->lookupTransform("/map", "/base_link", ros::Time(0), base_to_map);
//
//        Eigen::Matrix4f eigen_transform;
//        pcl_ros::transformAsMatrix (base_to_map, eigen_transform);
//        pcl_ros::transformPointCloud (eigen_transform, pointlcoud2, pointlcoud2);

        sensor_msgs::convertPointCloud2ToPointCloud(pointlcoud2, pointlcoud);

        pointlcoud.header = pointlcoud2.header;
        //pointlcoud.header.frame_id = "map";
        pub.publish(pointlcoud);
    }

}



void callback_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    datain_count ++;
       
    sensor_msgs::PointCloud2 cloud_odom;

    tf::StampedTransform velodyne_to_odom, odom_to_base;

    tfListener->waitForTransform("/odom", cloud_in->header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tfListener->lookupTransform("/odom", cloud_in->header.frame_id, ros::Time(0), velodyne_to_odom);
    
    // tfListener->waitForTransform("/odom", "base_link", ros::Time::now(), ros::Duration(1.0));
    // tfListener->lookupTransform("/odom", "base_link", ros::Time(0), odom_to_base);    

    Eigen::Matrix4f eigen_transform, eigen_transform_ob;
    pcl_ros::transformAsMatrix (velodyne_to_odom, eigen_transform);
   // pcl_ros::transformAsMatrix (odom_to_base, eigen_transform_ob);
    
    // cout << eigen_transform_ob(0, 3)<< " " <<  eigen_transform_ob(1, 3) << " " << eigen_transform_ob(2, 3) << endl;
    
    pcl_ros::transformPointCloud (eigen_transform, *cloud_in, cloud_odom);

    cloud_odom.header.frame_id = "odom";

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_odom, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_in_rgb;
    copyPointCloud(pcl_cloud, cloud_in_rgb);

    cloud_assambled += cloud_in_rgb;
    
    if(datain_count > 80)
    {
        datain_count = 0;
    }
    else
        return;
        
    cout << "cloud ready" << endl;

    pcl::PointCloud<pcl::Normal>::Ptr       normal      (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_prt   (new pcl::PointCloud<pcl::PointXYZRGB>(cloud_assambled));

/////// normal process ///////////////////////////////
    normal = calculateSurfaceNormal(cloud_prt, cloud_prt, 0.2);
    cout << "cloud normal finish" << endl;
    for(int i = 0; i < cloud_assambled.points.size(); i++)
    {
        float z = normal->points[i].normal[2];
        float color = 255* (1-z);
            
        cloud_assambled.points[i].r = color;
    }
    
    
        cout << "cloud color finish" << endl;
    // cloud_assambled = toImage.obstaclePointsDetection(cloud_assambled, eigen_transform_ob(0, 3), eigen_transform_ob(1, 3), *normal);
    // publish(pub_allclouds, toImage.m_all_points);
    // cout << "..... finished obstacle detect  " << ros::Time::now() << endl;

    // cloud_assambled = toImage.resampleCloud(cloud_assambled, robot_x, robot_y);
    
    cloud_assambled.header.frame_id = "odom";
    publish(pub_allclouds, cloud_assambled);
    cloud_assambled = cloud_clean;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "assambled_cloud_classifier");

    ros::NodeHandle node;
    ros::Rate rate(10.0);

    ros::Subscriber sub_velodyne = node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_ground", 1, callback_velodyne);
    //ros::Subscriber sub_assenbled = node.subscribe<sensor_msgs::PointCloud2>("/assembled_cloud", 1, callback_velodyne);

    pub_allclouds = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1);

    tfListener = new (tf::TransformListener);

    while (node.ok())
    {

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
