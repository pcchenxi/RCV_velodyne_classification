#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "terrain_function_sturcture.h"

using namespace std;

class Filter_Continuity
{
    public:
        float max_continuity;
        
        Filter_Continuity(int num_one_set );
        int point_num_h;
        Feature* filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set);
        Feature** filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);
        
        float get_varience( Feature *feature_set, int start_index, int size);
        float get_varience_dist( Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start_index, int size);
        float get_varience_height( Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start_index, int size);
        
        void  get_min_max_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set
                , int start_index, int size, float &min_height, float &max_height);
                
        pcl::PointCloud<pcl::PointXYZRGB> color_one_set(pcl::PointCloud<pcl::PointXYZRGB>  velodyne_sets, Feature  *feature_set);
        pcl::PointCloud<pcl::PointXYZRGB> color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);
        
    
};

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets, Feature *feature_set)
{   
    //cout << max_continuity << endl;
    float color_step = 255 * 255;
    for(int i = 0; i<720; i++)
    {
        if(feature_set[i].radius == 0)
            continue;
        
        float continuity_prob = feature_set[i].continuity_prob;
             
        float color = continuity_prob/max_continuity * 255;      
             
        //cout << continuity_prob << endl;     
        // float color = continuity_prob*255 * 10;     
        // int b = color / color_step;
        // float color_left = color - b*color_step;
        // int g = color_left / 255;
        // int r = color_left - g*255;
        //velodyne_sets.points[i].r = continuity_prob/max_continuity * 255;
        
        velodyne_sets.points[i].r = color;
        // velodyne_sets.points[i].g = g;
        // velodyne_sets.points[i].b = b;
    }
    
    return velodyne_sets;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
    pcl::PointCloud<pcl::PointXYZRGB> result;
    
    for(int i = 0; i<16; i++)
    {
        velodyne_sets[i] = color_one_set(velodyne_sets[i], feature_set[i]);
        result += velodyne_sets[i];
    }
    
    return result;
}


Filter_Continuity::Filter_Continuity(int num_one_set = 720)
{
    max_continuity = 0;
    point_num_h = num_one_set;
}

void Filter_Continuity::get_min_max_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set
                , int start_index, int size, float &min_height, float &max_height)
{
    min_height = 999;
    max_height = -999;
    
    int start = start_index - size/2;
    int end = start_index + size/2 + 1;
    
    if(start < 0)
        start = start + velodyne_set.points.size();
    if(end > velodyne_set.points.size()-1)
        end = velodyne_set.points.size()-1;
    
    for(int i = start; i < end; i++)
    {
        int index = i % velodyne_set.points.size();
        
        if(velodyne_set.points[index].x == 0 && velodyne_set.points[index].y == 0)
            continue;
            
        float height = velodyne_set.points[index].z;
        
        if(height < min_height)
            min_height = height;
        if(height > max_height)
            max_height = height;
    }
}

float Filter_Continuity::get_varience_height( Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start_index, int size)
{
    float sum = 0;
    float varience = 0;
    
    float avg = feature_set[start_index].radius;
    float avg_x = velodyne_set.points[start_index].x;
    float avg_y = velodyne_set.points[start_index].y;
    float avg_z = velodyne_set.points[start_index].z;
    
    int start = start_index - size/2;
    int end = start_index + size/2 + 1;
    
    if(start < 0)
        start = start + point_num_h;
    if(end > velodyne_set.points.size()-1)
        end = velodyne_set.points.size()-1;
    
    for(int i = start; i < end; i++)
    {
        int index = i % point_num_h;
        // float x = velodyne_set.points[i].x;
        // float y = velodyne_set.points[i].y;
        float z = velodyne_set.points[index].z;
        
        // float diff_x = abs(x - avg_x);
        // float diff_y = abs(y - avg_y);
        float diff_z = abs(z - avg_z);
        
        if(diff_z > 0.1)
            varience =1;
    }
    
    //cout << "varience: " << varience << endl;
    return varience;    
    
}

float Filter_Continuity::get_varience_dist( Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start_index, int size)
{
    float sum = 0;
    float varience = 0;
    
    int step = size/2;
    int start = 0, end = 0;
    
    int count = 0;
    for(int i = start_index; i > 0; i--)
    {
        float r = feature_set[i].radius;
        if(r == 0)
            continue;
        
        count ++;
        start = i;
        if(count == step)
            break;    
    }
    
    count = 0;
    for(int i = start_index; i < velodyne_set.points.size()-1; i++)
    {
        float r = feature_set[i].radius;
        if(r == 0)
            continue;
        
        count ++;
        end = i;
        if(count == step)
            break;    
    }
    
    float avg = feature_set[start_index].radius;
    
    if(start < 0)
        start = start + point_num_h;
    if(end > velodyne_set.points.size()-1)
        end = velodyne_set.points.size()-1;
    
    float start_x = velodyne_set.points[start].x;
    float start_y = velodyne_set.points[start].y;
    float start_r = feature_set[start].radius;
    
    float end_x = velodyne_set.points[end].x;
    float end_y = velodyne_set.points[end].y;
    float end_r = feature_set[end].radius;    
    
    float avg_r = (start_r + end_r)/2;
    
    float y0 = (end_y - start_y);
    float x0 = (end_x - start_x);
    
    float k = y0/x0;
    
    float x = velodyne_set.points[start_index].x;
    float y = velodyne_set.points[start_index].y;
    float r = feature_set[start_index].radius;
    
    float diff_x = x - start_x;
    float diff_y = y - start_y;
    
    float s = sqrt(1 + k*k);
    
    float dist = abs(k*diff_x - diff_y)/s;    
    
    if(r < avg_r && dist > 0.15)
        varience = 1;
    
    
    // // check the slope change
    // float angle = atan2(y0, x0) * 180 / 3.1415926 + 180;
        
    // feature_set[start_index].slope = angle;
    
    // int index_pre = start_index -1;
    // if(index_pre < 0)
    //     index_pre = 0;
    
    // float slope_pre = feature_set[index_pre].slope;
    // float diff_slope = abs(slope_pre - angle);
    
    // if(diff_slope > 180)
    //     diff_slope = 360 - diff_slope;
    // if(diff_slope > 90)
    //     diff_slope = 180 - diff_slope;
        
    // cout << "diff_slope angle: " << start_index << " "<< diff_slope << " " << slope_pre << " " << angle << endl;
    
    // if(r < avg_r && diff_slope > 15)
    //     varience = 1;
    
    
    return varience;
}

float Filter_Continuity::get_varience( Feature *feature_set, int start_index, int size)
{
    float sum = 0;
    float varience = 0;
    int   valud_count = 0;
    
    float avg = feature_set[start_index].radius;
    
    int start = start_index - size/2;
    int end = start_index + size/2 + 1;
    
    if(start < 0)
        start = start + point_num_h;
    if(end >= point_num_h)
        end = point_num_h - 1;
    
    //cout << "start: " << start << "  end: " << end << endl; 
    for(int i = start; i < end; i++)
    {
        int index = i % point_num_h;
        float r = feature_set[index].radius;
        //cout << "avg: " << avg << "  r: " << r << endl;    
        if(r == 0)
            continue;
            
        valud_count ++;  
            
        float diff = abs(r - avg);
       // cout << diff <<"        " << r << " " << avg << endl;
        if(diff > 0.15)
        {
            varience = 1;
            break;
        }    
    }
    
    if(valud_count < 2)
        varience = 1;
  //  cout << "varience: " << varience << endl;
    return varience;
}

Feature ** Filter_Continuity::filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
        max_continuity = 0;
    pcl::PointCloud<pcl::PointXYZRGB> result;
    for(int i = 0; i<10; i++)
    {
        feature_set[i] = filtering_one_set(velodyne_sets[i], feature_set[i]);
        //result += velodyne_sets[i];
    }
    
    return feature_set;
}

pcl::PointXYZRGB set_point_color(pcl::PointXYZRGB point, int r, int g, int b)
{
    point.r = r;
    point.g = g;
    point.b = b;
    
    return point;
}

Feature * Filter_Continuity::filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set)
{
    int start_index = 0;
    int pre_index = 0;
    float color_step = 255 * 255;
    
    for(int i = 0; i < velodyne_set.points.size(); i = i+1)
    {
        float varience = 0;
        if(feature_set[i].radius == 0)
            continue;
        
        float min_height, max_height;
        
        // get_min_max_height(velodyne_set, i, 5, min_height, max_height);
        // float diff_height   = abs( min_height - max_height);
        
        // cout << diff_height << endl;
        // if(diff_height > 0.15)
        //     varience = 1;
            
        // float varience_1 = get_varience_dist(feature_set, velodyne_set, i, 9);    
        // float varience_2 = get_varience_dist(feature_set, velodyne_set, i, 5);    
        // if(varience_1 == 1 || varience_2 == 1)
        //     varience = 1;
        
        varience = get_varience(feature_set, i, 5);
        //varience = get_varience_dist(feature_set, velodyne_set, i, 5);
        //varience = get_varience_height(feature_set, velodyne_set, i, 7);
        
        
        feature_set[i].continuity_prob = varience;
        
        
        if(varience > max_continuity)
            max_continuity = varience;
        
        //cout << "height: " << diff_height << endl;
        
        // float color = diff_height*255 * 10;
        // int b = color / color_step;
        // float color_left = color - b*color_step;
        // int g = color_left / 255;
        // int r = color_left - g*255;
        
        // velodyne_set.points[i] = set_point_color(velodyne_set.points[i], r, g, b);
        
        // if(diff_height < 1)
        // {
        //     velodyne_set.points[i] = set_point_color(velodyne_set.points[i], 0, 255, 0);
        //     //velodyne_set.points[pre_index] = set_point_color(velodyne_set.points[pre_index], 0, 255, 0);
        // }    
        // else 
        // {
        //     velodyne_set.points[i] = set_point_color(velodyne_set.points[i], 255, 0, 0);
        //     //velodyne_set.points[pre_index] = set_point_color(velodyne_set.points[pre_index], 255, 0, 0);
        // }
    }
    //cout << "max: " << max_continuity;
    //cout << endl;
    return feature_set;
}