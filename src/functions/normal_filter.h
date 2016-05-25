#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d_omp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "terrain_function_sturcture.h"

using namespace std;
using namespace cv;

class Filter_Normal
{
    public:
        float max_continuity;
        
        Filter_Normal(int num_one_set );
        
        int point_num_h;
        Feature* filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set);
        Feature** filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);
                
        pcl::PointCloud<pcl::PointXYZRGB> color_one_set(pcl::PointCloud<pcl::PointXYZRGB>  velodyne_sets, Feature  *feature_set);
        pcl::PointCloud<pcl::PointXYZRGB> color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);
            
        pcl::PointXYZRGB set_point_color(pcl::PointXYZRGB point, int r, int g, int b);
        
        pcl::PointCloud<pcl::Normal>::Ptr calculateSurfaceNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point,
                                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr search_point,
                                                         float searchRadius );
};

pcl::PointCloud<pcl::Normal>::Ptr Filter_Normal::calculateSurfaceNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point,
                                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr search_point,
                                                         float searchRadius )
{
    //cout << "normal calculation" << endl;
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

pcl::PointCloud<pcl::PointXYZRGB> Filter_Normal::color_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets, Feature *feature_set)
{   
    //cout << "in filtering one color" << endl;
    for(int i = 0; i<velodyne_sets.points.size(); i++)
    {
        if(feature_set[i].radius == 0)
            continue;
            
        float normal = feature_set[i].normal;
             
        float color = normal*255;     
        
        if(normal > 0)
            velodyne_sets.points[i].r = color;
        else
            velodyne_sets.points[i].g = color;
    }
    
    return velodyne_sets;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Normal::color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
    //cout << "in filtering all color" << endl;
    max_continuity = 0;
    pcl::PointCloud<pcl::PointXYZRGB> result;
    
    for(int i = 0; i<16; i++)
    {
        velodyne_sets[i] = color_one_set(velodyne_sets[i], feature_set[i]);
        result += velodyne_sets[i];
    }
    
    return result;
}


Filter_Normal::Filter_Normal(int num_one_set = 720)
{
    point_num_h = num_one_set;
}


Feature ** Filter_Normal::filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
    //cout << "in normal filter" << endl;
    for(int i = 0; i<16; i++)
    {
        //cout << i << endl;
        feature_set[i] = filtering_one_set(velodyne_sets[i], feature_set[i]);
    }
    
    return feature_set;
}

pcl::PointXYZRGB Filter_Normal::set_point_color(pcl::PointXYZRGB point, int r, int g, int b)
{
    point.r = r;
    point.g = g;
    point.b = b;
    
    return point;
}

Feature * Filter_Normal::filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set)
{
    //cout << "in filtering one set" << endl;
    pcl::PointCloud<pcl::Normal>::Ptr       normal      (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_prt   (new pcl::PointCloud<pcl::PointXYZRGB>(velodyne_set));

    normal = calculateSurfaceNormal(cloud_prt, cloud_prt, 1.0);
    //cout << "..... finished                  " << ros::Time::now() << endl;
    
    for(int i = 0; i < velodyne_set.points.size(); i++)
    {
        feature_set[i].normal = normal->points[i].normal[2];
    }
    return feature_set;
}