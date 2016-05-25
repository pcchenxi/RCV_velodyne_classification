#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "terrain_function_sturcture.h"

using namespace std;
using namespace cv;

class Filter_Histogram
{
    public:
        float max_continuity;
        
        Filter_Histogram(int num_one_set );
        
        int point_num_h;
        Feature* filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set);
        Feature** filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);
                
        pcl::PointCloud<pcl::PointXYZRGB> color_one_set(pcl::PointCloud<pcl::PointXYZRGB>  velodyne_sets, Feature  *feature_set);
        pcl::PointCloud<pcl::PointXYZRGB> color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);
        
        
        Mat transform_to_mat(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set);
        Mat get_histogram(Mat img);
        pcl::PointXYZRGB set_point_color(pcl::PointXYZRGB point, int r, int g, int b);
        void draw_histogram(Mat hist, int his_size);
        
        
        pcl::PointCloud<pcl::PointXYZRGB>  get_hist_from_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud);
    
};

pcl::PointCloud<pcl::PointXYZRGB> Filter_Histogram::color_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets, Feature *feature_set)
{   
    float color_step = 255 * 255;
    for(int i = 0; i<720; i++)
    {
        if(feature_set[i].radius == 0)
            continue;
            
        float continuity_prob = feature_set[i].continuity_prob;
             
        float color = continuity_prob*255 * 10;     
        int b = color / color_step;
        float color_left = color - b*color_step;
        int g = color_left / 255;
        int r = color_left - g*255;
        //velodyne_sets.points[i].r = continuity_prob/max_continuity * 255;
        
        velodyne_sets.points[i].r = r;
        velodyne_sets.points[i].g = g;
        velodyne_sets.points[i].b = b;
    }
    
    return velodyne_sets;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Histogram::color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
    max_continuity = 0;
    pcl::PointCloud<pcl::PointXYZRGB> result;
    
    for(int i = 0; i<16; i++)
    {
        velodyne_sets[i] = color_one_set(velodyne_sets[i], feature_set[i]);
        result += velodyne_sets[i];
    }
    
    return result;
}


Filter_Histogram::Filter_Histogram(int num_one_set = 720)
{
    point_num_h = num_one_set;
}


Mat Filter_Histogram::transform_to_mat(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set)
{
    int point_num      = velodyne_set.points.size();
    Mat pointset_mat   = Mat(1, point_num, CV_32FC1,  Scalar(0));
    
    for(int i = 0; i<point_num; i++)
    {
        float z = velodyne_set.points[i].z;
        pointset_mat.ptr<float>(0)[i] = z;
        //cout << z << " ";
    }
    
    //cout << endl;
    //imshow("cloud set", pointset_mat );
    
    waitKey(10);
    return pointset_mat;
}

void Filter_Histogram::draw_histogram(Mat hist, int his_size)
{
    cout << "   in draw histogram " << endl;
    int hist_w = 512; int hist_h = 720;
    int bin_w = cvRound( (double) hist_w/his_size );
    
    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    
    /// Normalize the result to [ 0, histImage.rows ]
    //normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    
    /// Draw for each channel
    for( int i = 1; i < his_size; i++ )
    {
       // cout << cvRound(hist.at<float>(i-1)) << " ";
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                        Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                        Scalar( 255, 0, 0), 2, 8, 0  );
                        
        //cout << cvRound(hist.at<float>(i-1)) << " ";
    }
    
    cout << endl;
    /// Display
   namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
   imshow("calcHist Demo", histImage );
    
   waitKey(10);
    
}

Mat Filter_Histogram::get_histogram(Mat img)
{
   // cout << "   in get histogram " << endl;    
    float step = 0.01;
    float max_height = 0.3;
    float min_height = 0.0;
    float diff_height = max_height - min_height;
    
    int histSize = diff_height/step +1;
    float range[] = {min_height, max_height }; 
    const float* histRange = { range };
    bool uniform = true; 
    bool accumulate = false;
    
    Mat hist;
    
    calcHist( &img, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
    draw_histogram(hist, histSize);
    
    return hist;
}

Feature ** Filter_Histogram::filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
    //cout << "in histogram filter" << endl;
    pcl::PointCloud<pcl::PointXYZRGB> result;
    for(int i = 0; i<17; i++)
    {
        cout << i << endl;
        Mat pointset_mat = transform_to_mat(velodyne_sets[i]);
        Mat hist = get_histogram(pointset_mat);
        //feature_set[i] = filtering_one_set(velodyne_sets[i], feature_set[i], Mat hist);
        //result += velodyne_sets[i];
    }
    
    return feature_set;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Histogram::get_hist_from_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB> result;
    
    Mat pointset_mat = transform_to_mat(cloud);
    Mat hist = get_histogram(pointset_mat);
    
    double min_v, max_v = 0;
    Point min_index, max_index;
    minMaxLoc(hist, &min_v, &max_v, &min_index, &max_index);
    
    cout << "max his value: " << max_v << " index: " << max_index.y<< endl;
    float height = 0.01 * max_index.y + 0.01;
    
    for(int i = 0; i<cloud.points.size(); i++)
    {
        float z = cloud.points[i].z;
        if(z < height)
            result.points.push_back(cloud.points[i]);
    }
    
    return cloud;
}

pcl::PointXYZRGB Filter_Histogram::set_point_color(pcl::PointXYZRGB point, int r, int g, int b)
{
    point.r = r;
    point.g = g;
    point.b = b;
    
    return point;
}

Feature * Filter_Histogram::filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set)
{
    int start_index = 0;
    int pre_index = 0;
    float color_step = 255 * 255;
    
    for(int i = 0; i < velodyne_set.points.size(); i = i+1)
    {
        if(feature_set[i].radius == 0)
            continue;
      
    }

    return feature_set;
}