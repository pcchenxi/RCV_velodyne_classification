#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/impl/transforms.hpp"
#include <pcl/features/normal_3d_omp.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace cv;

float height_step = 0.1;
int height_num = 6/height_step;

class Cloud_Image_Convertor
{
    public:
    pcl::PointCloud<pcl::PointXYZRGB> m_all_points, m_obs_points, m_costmap_points;

    vector<Mat> m_img_allheight;
    Mat     m_img_max, m_img_min, m_img_diff, m_img_mean, m_img_density, m_img_gra, m_img_closed, m_img_costmap, m_img_obs;
    Mat     m_velodyne_density;
    Mat     m_database_density;

    uchar   **m_costmap_char;  // 0: unknown     1~254: cost value      255: obstacle

    float   m_map_width; // 20 Meter
    float   m_map_depth; // 20 Meter
    float   m_pix_size;    // 5 cm
    int     m_image_rows;
    int     m_image_cols;

    float   m_min_z;

    Cloud_Image_Convertor(float map_width, float map_depth, float pix_size);
    ~Cloud_Image_Convertor();

    Mat  generate_database_density();
    void init_img(int image_rows, int image_cols, float init_value);

    pcl::PointCloud<pcl::PointXYZRGB> scale_cloud(pcl::PointCloud<pcl::PointXYZRGB> clouds, float robot_x, float robot_y);

    pcl::PointCloud<pcl::PointXYZRGB> obstaclePointsDetection(pcl::PointCloud<pcl::PointXYZRGB> cloud, float robot_x, float robot_y, pcl::PointCloud<pcl::Normal> normal);
    pcl::PointCloud<pcl::PointXYZRGB> obstaclePointsDetection_singlescene(pcl::PointCloud<pcl::PointXYZRGB> cloud, float robot_x, float robot_y);

    Mat set_pix_Value(pcl::PointCloud<pcl::PointXYZRGB> cloud);
    pcl::PointXYZRGB set_point_color(pcl::PointXYZRGB point, int r, int g, int b);

    float is_Good_Point(vector<Mat> img_all, int row, int col, float height, float dist);
    float get_goodness(vector<Mat> img_all, int row, int col, float height, float dist);

    int is_blocked(vector<Mat> img_all, int row, int col, float height, float dist);
    Mat remove_edgeSobel(Mat img);
    bool isEdge(Mat img, int row, int col, int size);
    Mat generate_costmap(Mat img_close, Mat img_gre, Mat img_obs_pre, float thresh_walk, float thresh_obs);


    pcl::PointCloud<pcl::PointXYZRGB> getCloutCostMap(Mat img_cost, Mat img_closed, float robot_x, float robot_y);


    pcl::PointCloud<pcl::PointXYZRGB> resampleCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, float robot_x, float robot_y);
    pcl::PointCloud<pcl::PointXYZRGB> label_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, Mat close_img, Mat label_img, Mat index_img, float thresh_walk, float thresh_obs);
    pcl::PointCloud<pcl::PointXYZRGB> label_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, Mat img_costmap, Mat index_img);


    // pose selection
    geometry_msgs::PoseStamped select_first_stoppose(nav_msgs::Path path, float robot_x, float robot_y);

    // image interpolatein
    Mat interpolate_image(Mat img);
    float get_interpolated_value(Mat img, int row, int col, int size);
};


geometry_msgs::PoseStamped Cloud_Image_Convertor::select_first_stoppose(nav_msgs::Path path, float robot_x, float robot_y)
{
    int pose_num = path.poses.size();
    bool need_stop = false;
    cout << "pose_num: " << pose_num << endl;
    geometry_msgs::PoseStamped pose;

    int noinfo_index = 0;
    for(int i = 0; i<pose_num; i++)
    {
        float pose_x = path.poses[i].pose.position.x;
        float pose_y = path.poses[i].pose.position.y;

        float diff_x = abs(robot_x - pose_x);
        float diff_y = abs(robot_y - pose_y);

        float dist = sqrt(diff_x*diff_x + diff_y*diff_y);

        if(dist < 2.0)
            continue;
        else if(dist > 5.0)
        {
            noinfo_index = i;
            need_stop = true;
            break;
        }
        else
        {
            float scaled_x = pose_x - robot_x + m_map_width/2;
            float scaled_y = pose_y - robot_y + m_map_depth/2;

            int index_x = (int)(scaled_x/m_pix_size);
            int index_y = (int)(scaled_y/m_pix_size);

            int pix_row = index_y;
            int pix_col = index_x;

            float cost_v = m_img_costmap.ptr<uchar>(pix_row)[pix_col];
            if(cost_v == 0)
            {
                noinfo_index = i;
                need_stop = true;
                break;
            }
        }
    }

    int stop_index = noinfo_index - 50;
    cout << "costmap_generator: stop index: " << noinfo_index << "  " << stop_index <<" " << need_stop<< endl;

    if(stop_index < 0)
        stop_index = 0;

    if(!need_stop)
    {
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        cout << "no need to stop" << endl;
    }
    else
    {
       // pose.pose.position.x = stop_index;
        pose = path.poses[stop_index];
        //pose = path.poses[stop_index];
    }

 //       imshow("costmap", m_img_costmap);
//    waitKey(0);
    return pose;
}

pcl::PointCloud<pcl::PointXYZRGB> Cloud_Image_Convertor::getCloutCostMap(Mat img_cost, Mat img_closed, float robot_x, float robot_y)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    float shift_x = (robot_x - m_map_depth/2)/m_pix_size;
    float shift_y = (robot_y - m_map_width/2)/m_pix_size;

    for(int i = 0; i<m_image_rows; i++) // y
    for(int j = 0; j<m_image_cols; j++) // x
    {
        float x_p = (float)j + shift_x;
        float y_p = (float)i + shift_y;

        float cost_v = img_cost.ptr<uchar>(i)[j];
        float height_v = img_closed.ptr<uchar>(i)[j];
        if(cost_v == 0) // no information should be 255 in n
            cost_v = 255;
        else if(cost_v == 255)  // obstacles, should be 254 in planner
            cost_v = 254;

        pcl::PointXYZRGB point, point2;
        point.x = x_p*m_pix_size - 0.02;
        point.y = y_p*m_pix_size - 0.02;
        //point.z = height_v;
        point.z = cost_v;

        point2.x = x_p*m_pix_size + 0.02;
        point2.y = y_p*m_pix_size + 0.02;
        //point2.z = height_v;
        point2.z = cost_v;

        cloud.points.push_back(point);
        cloud.points.push_back(point2);
    }

    cout << "shift_x: " << shift_x << "  y: " << shift_y << endl;
    return cloud;
}


Mat Cloud_Image_Convertor::generate_costmap(Mat img_close, Mat img_gre, Mat img_obs_pre, float thresh_walk, float thresh_obs)
{
    Mat cost_map    = Mat(m_image_rows,m_image_cols, CV_8UC1,  Scalar(0));
    Mat obs_map     = Mat(m_image_rows,m_image_cols, CV_8UC1,  Scalar(0));
    Mat ground_map  = Mat(m_image_rows,m_image_cols, CV_8UC1,  Scalar(0));

    for(int i = 0; i<m_image_rows; i++) // y
    for(int j = 0; j<m_image_cols; j++) // x
    {
        float close_v = img_close.ptr<float>(i)[j];
        float gre_v   = img_gre.ptr<float>(i)[j];
        float pix_v   = gre_v/thresh_obs * 254 + 1;
        uchar obs_v   = img_obs_pre.ptr<float>(i)[j];

        if(obs_v == 3)     // set the base obstacle based on the preselection result
            ground_map.ptr<uchar>(i)[j] = 255;
        if( close_v != 0 && gre_v < thresh_walk)   // 0 unknown, 255 obstacle, rest: cost value
            ground_map.ptr<uchar>(i)[j] = pix_v;
        else if(close_v != 0 && gre_v > thresh_walk)
            obs_map.ptr<uchar>(i)[j] = 255;
        else if(close_v != 0 && gre_v == thresh_walk)
            obs_map.ptr<uchar>(i)[j] = 255;

    }

    //Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    //morphologyEx(obs_map, obs_map, MORPH_DILATE, element);

    cost_map = obs_map + ground_map;

    for(int i = 0; i<m_image_rows; i++) // y
    for(int j = 0; j<m_image_cols; j++) // x
    {
        uchar cost_v = cost_map.ptr<uchar>(i)[j];
        m_costmap_char[i][j] = cost_v;
    }

    return cost_map;
}


pcl::PointCloud<pcl::PointXYZRGB> Cloud_Image_Convertor::label_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, Mat img_costmap, Mat index_img)
{
    for(int i = 0; i<cloud.points.size(); i++)
    {
        int row = index_img.ptr<float>(i)[0];
        int col = index_img.ptr<float>(i)[1];

        uchar cost_v = img_costmap.ptr<uchar>(row)[col];

        if(cost_v == 0)
        {
            cloud.points[i] = set_point_color(cloud.points[i], 0, 255, 0); // unknown
            continue;
        }

        if(cost_v == 255)  // obs
        {
            cloud.points[i] = set_point_color(cloud.points[i], 255, 0, 0);
        }
        else
        {
            cloud.points[i] = set_point_color(cloud.points[i], 0, 0, cost_v); // drivable
        }
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB> Cloud_Image_Convertor::label_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, Mat close_img, Mat label_img, Mat index_img, float thresh_walk, float thresh_obs)
{
    for(int i = 0; i<cloud.points.size(); i++)
    {
        int row = index_img.ptr<float>(i)[0];
        int col = index_img.ptr<float>(i)[1];

        float num = label_img.ptr<float>(row)[col];
        float close_v = close_img.ptr<float>(row)[col];

        if(close_v == 0)
        {
            cloud.points[i] = set_point_color(cloud.points[i], 0, 0, 0); // unknown
            continue;
        }

        if(num < thresh_walk)
        {
            float s = num/thresh_walk;
            cloud.points[i] = set_point_color(cloud.points[i], 0, 255*s, 0); // can drive, darker the best
            //cloud.points[i] = set_point_color(cloud.points[i], 0, 0, 0);
        }
        else if(num < thresh_obs)
        {
            float s = num/thresh_obs;
            cloud.points[i] = set_point_color(cloud.points[i], 255*s, 255, 0); // need walk or slope
            //cloud.points[i] = set_point_color(cloud.points[i], 0, 254, 0);
        }
        else
        {
            cloud.points[i] = set_point_color(cloud.points[i], 255, 0, 0); // next to obs
            //cloud.points[i] = set_point_color(cloud.points[i], 0, 254, 0);
        }
    }

    return cloud;
}

Cloud_Image_Convertor::Cloud_Image_Convertor(float map_width = 40, float map_depth = 40, float pix_size = 0.05)
{
    m_map_width = map_width; // 20 Meter
    m_map_depth = map_depth; // 20 Meter
    m_pix_size  = pix_size;    // 5 cm

    m_image_rows = map_width / pix_size;
    m_image_cols = map_depth / pix_size;

    m_costmap_char = new uchar*[m_image_rows];

    //m_database_density = generate_database_density();
}

Cloud_Image_Convertor::~Cloud_Image_Convertor()
{
    m_img_allheight.clear();

    if(m_costmap_char != NULL)
        delete[] m_costmap_char;
}


pcl::PointCloud<pcl::PointXYZRGB> Cloud_Image_Convertor::scale_cloud(pcl::PointCloud<pcl::PointXYZRGB> clouds, float robot_x, float robot_y)
{
    m_min_z         = 0;

    for(int i = 0; i<clouds.points.size(); i++)
    {
        float x = clouds.points[i].x;
        float y = clouds.points[i].y;

        float scaled_x = x - robot_x + m_map_width/2;
        float scaled_y = y - robot_y + m_map_depth/2;

        if(scaled_x > m_map_width-0.1 || scaled_y > m_map_depth -0.1 || scaled_x < 0 || scaled_y < 0)
        {
            clouds.points[i].x = 0;
            clouds.points[i].y = 0;
            continue;
        }   
        
        clouds.points[i].x = scaled_x;
        clouds.points[i].y = scaled_y;
        //clouds.points[i].z = clouds.points[i].z + 1;
        
        if(clouds.points[i].z < m_min_z)
            m_min_z = clouds.points[i].z;
    }

    return clouds;
}

void Cloud_Image_Convertor::init_img(int image_rows, int image_cols, float init_value)
{
    m_velodyne_density  = Mat(m_image_rows,m_image_cols, CV_8UC1,  Scalar(0));
    m_img_max           = Mat(m_image_rows,m_image_cols, CV_32FC1, Scalar(0.0));
    m_img_min           = Mat(m_image_rows,m_image_cols, CV_32FC1, Scalar(0.0));

    Mat occup_map       = Mat(500, 500, CV_8UC1,  Scalar(205));

    m_img_allheight.clear();
    m_costmap_points.points.clear();

    for(int i = 0; i< height_num; i++)
    {
        Mat a = Mat(m_image_rows,m_image_cols, CV_32FC1, Scalar(0.0));
        m_img_allheight.push_back(a);
    }

    for(int i = 0; i < m_image_rows; i++)
    {
        m_costmap_char[i] = new uchar[m_image_cols];
    }

    m_img_diff      = Mat(image_rows,image_cols,CV_32FC1, Scalar(0.0));
    m_img_mean      = Mat(image_rows,image_cols,CV_32FC1, Scalar(0.0));
    m_img_density   = Mat(image_rows,image_cols,CV_32FC1, Scalar(1.0));
    m_img_gra       = Mat(image_rows,image_cols,CV_32FC1, Scalar(0.0));

    m_min_z         = 0;


}

Mat normalizeMat(Mat img, float max_value = 1.0, float *scale_value = NULL)
{
    double min_v, max_v = 0;
    minMaxLoc(img, &min_v, &max_v);

    img = img - min_v;

    cout << "max_v: " << max_v << " min_v: "<< min_v << endl;
    float scale = max_value/(max_v-min_v);
    Mat a = img * scale;

    if(scale_value != NULL)
        *scale_value = scale;

    return a;
}


Mat Cloud_Image_Convertor::set_pix_Value(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    cout << cloud.points.size() << endl;
    Mat index_max(cloud.points.size(), 2, CV_32FC1, Scalar(0.0));
    cout << "   init index image " << endl;

    for(int i = 0; i<cloud.points.size(); i++)
    {
        if(cloud.points[i].x == 0 && cloud.points[i].y == 0)
            continue;
            
        pcl::PointXYZRGB point = cloud.points[i];

        point.z = point.z - m_min_z + 0.7;

        int index_x = (int)(point.x/m_pix_size);
        int index_y = (int)(point.y/m_pix_size);

        int pix_row = index_y;
        int pix_col = index_x;

        index_max.ptr<float>(i)[0] = pix_row;
        index_max.ptr<float>(i)[1] = pix_col;

        uchar point_density = m_velodyne_density.ptr<uchar>(pix_row)[pix_col];

        if(point_density < 255)
            m_velodyne_density.ptr<uchar>(pix_row)[pix_col] = point_density + 1;

        //// image max
        float max_height = m_img_max.ptr<float>(pix_row)[pix_col];
        if(point.z > max_height)
            m_img_max.ptr<float>(pix_row)[pix_col] = point.z;

        //// image min
        float min_height = m_img_min.ptr<float>(pix_row)[pix_col];
        if(point.z < min_height || min_height == 0.0)
            m_img_min.ptr<float>(pix_row)[pix_col] = point.z;

        //// image height
        float height_index = point.z/height_step;
        if(height_index > height_num-1)
            height_index = height_num-1;
        if(height_index < 0)
            height_index = 0;
        (m_img_allheight[height_index]).ptr<float>(pix_row)[pix_col] += 1;
    }

    return index_max;
}

pcl::PointXYZRGB Cloud_Image_Convertor::set_point_color(pcl::PointXYZRGB point, int r, int g, int b)
{
    point.r = r;
    point.g = g;
    point.b = b;

    return point;
}


float Cloud_Image_Convertor::get_goodness(vector<Mat> img_all, int row, int col, float height, float dist)
{
    if(row == 0 && col == 0)
    return 1;

    float error = 0;//0.02 * dist / height_step;

    float height_index = height/height_step;

    //// for walk over
    bool need_walk = false;
    int	 walk_num = 0;
    int start_index = height_index + 1 + error;
    int end_index = start_index + 4 + error;

    if(start_index < 0)
        start_index = 0;

    for(int i = start_index; i < end_index; i++)
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
	walk_num += num;
    }
    //cout << "finish need_walk  " << need_walk << endl;

    //// for blocking
    bool blocked = false;
    int blocked_num = 0;
    start_index = height_index + 4 + error;
    end_index = start_index + 8 + error;
    if(start_index < 0)
        start_index = 0;
    if(end_index > height_num)
        end_index = height_num;

    for(int i = start_index; i < end_index; i++)
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
	blocked_num += num;
    }
    //cout << "finish blocking  " << blocked << endl;


    //// for shunk
    bool need_shunk = false;
    int shunk_num = 0;
    start_index = height_index + 10 + error;

    if(start_index < 0)
        start_index = 0;

    if(start_index > height_num-1)
        start_index = height_num;

    end_index = start_index + 5 + error;
    if(end_index > height_num)
        end_index = height_num;

    for(int i = start_index; i < end_index; i++)
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
	shunk_num += num;
    }
    //cout << "finish shunk " << need_shunk << endl;



    //// for obs
    bool is_obs = false;


    float total_num = shunk_num + blocked_num + walk_num;
    float total_g = shunk_num * 0.2 + blocked_num*1.0 + walk_num * 0.2;
    float result = total_g/total_num;

    if(is_obs)
	result = 1;

    return result;

}

float Cloud_Image_Convertor::is_Good_Point(vector<Mat> img_all, int row, int col, float height, float dist)
{
    if(row == 0 && col == 0)
    return -1;

    float error = 0;//0.02 * dist / height_step;

    float height_index = height/height_step;
    int start_index, end_index;

    //// for blocking
    bool blocked = false;
    start_index = height_index + 4 + error;
    end_index = start_index + 8 + error;
    if(start_index < 0)
        start_index = 0;
    if(start_index > height_num-1)
        start_index = height_num;
    if(end_index > height_num)
        end_index = height_num;

    for(int i = start_index; i < end_index; i++)
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
        if(num != 0)
            return 1;
    }

    //cout << "blocking "  << endl;
    //// for edge
    bool is_edge = false;
    start_index = height_index - 3 ;
    if(start_index < 0)
        start_index = -10;
    for(int i = start_index; i > -1; i--)
    {
	if(start_index == -10)
	    break;

        float num = (m_img_allheight[i]).ptr<float>(row)[col];
        if(num != 0)
            return 1;
    }
    //cout << " edge "  << endl;

    int p_count = 0;
    start_index = height_index -1 + error;
    end_index = start_index + 3 + error;
    if(start_index < 0)
        start_index = 0;
    if(start_index > height_num-1)
        start_index = height_num;
    if(end_index > height_num)
        end_index = height_num;

    for(int i = start_index; i < end_index; i++)
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
        if(num != 0)
            p_count++;
    }
    if(p_count > 2)
	return 1;

//cout << " edge2 " << endl;
    //// for shunk
    bool need_shunk = false;
    start_index = height_index + 10 + error;

    if(start_index < 0)
        start_index = 0;

    if(start_index > height_num-1)
        start_index = height_num;

    end_index = start_index + 5 + error;
    if(end_index > height_num)
        end_index = height_num;

    for(int i = start_index; i < end_index; i++)
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
        if(num != 0)
            return 2;
    }
    //cout << "finish shunk " << endl;


    //// for walk over
    bool need_walk = false;

    start_index = height_index + 1 + error;
    end_index = start_index + 4 + error;

    if(start_index < 0)
        start_index = 0;
    if(start_index > height_num-1)
        start_index = height_num;
    if(end_index > height_num)
        end_index = height_num;
    for(int i = start_index; i < end_index; i++)
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
        if(num != 0)
            return 3;
    }
    //cout << "finish need_walk  " << endl;

    return 0;

}

int Cloud_Image_Convertor::is_blocked(vector<Mat> img_all, int row, int col, float height, float dist)
{
    if(row == 0 && col == 0)
    return 2;

    float height_index = height/height_step;
    int start_index, end_index;

    float block_height_1 = height_index + 0.8/height_step;
    float block_height_2 = height_index + 1.5/height_step;

    if(block_height_1 > height_num-1)
        block_height_1 = height_num;

    if(block_height_2 > height_num-1)
        block_height_2 = height_num;

    for(int i = height_index + 2; i < block_height_1; i++)  // blocking
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
        if(num > 0 )
            return 2;
    }

    for(int i = block_height_1; i < block_height_2; i++)  // need to shink
    {
        float num = (m_img_allheight[i]).ptr<float>(row)[col];
        if(num > 0 )
            return 1;
    }

    return 0;
}


bool Cloud_Image_Convertor::isEdge(Mat closed, int row, int col, int size)
{
    int d = size/2;

    for(int i = row - d; i<row + d; i++) // y
    for(int j = col - d; j<col + d; j++) // x
    {
        if(i < 0 || i-1 >m_image_rows ||
            j < 0 || j-1 >m_image_cols)
            continue;

        float num = closed.ptr<float>(i)[j];
        if(num == 0)
        {
            return true;
        }
    }

    return false;
}

Mat Cloud_Image_Convertor::remove_edgeSobel(Mat img)
{
    for(int i = 0; i<m_image_rows; i++) // y
    for(int j = 0; j<m_image_cols; j++) // x
    {
        bool e = isEdge(m_img_closed, i, j, 3);
        if(e)
        {
            img.ptr<float>(i)[j] = 0;
        }
    }

    return img;
}

float Cloud_Image_Convertor::get_interpolated_value(Mat img, int row, int col, int size)
{
    float v = img.ptr<float>(row)[col];
    if(v != 0)
        return v;


    int scale = size/2;
    int start_row  = row - scale;
    int finish_row = row + scale + 1;
    int start_col  = col - scale;
    int finish_col = col + scale + 1 ;

    float dist_sum = 0;

    float max_dist = scale*1.5;

    float *values = new float[size*size];
    float *dists  = new float[size*size];

    if(start_row < 0)
        start_row = 0;
    if(start_col < 0)
        start_col = 0;
    if(finish_row >m_image_rows-1)
        finish_row = m_image_rows-1;
    if(finish_col >m_image_cols-1)
        finish_col = m_image_cols-1;

    int index = 0;
    for(int i = start_row; i < finish_row; i++)
    {
        for(int j = start_col; j < finish_col; j++)
        {
            float value = img.ptr<float>(i)[j];
            if(value == 0)
            {
                values[index] = 0;
                dists[index] = 0;
                index ++;
                continue;
            }

            float diff_row = abs(i - row);
            float diff_col = abs(j - col);
            float diff = sqrt(diff_row*diff_row + diff_col*diff_col);

            float dist = 1 - diff/max_dist;

            if(diff_row < diff_col)
                dist = diff_col;

            dist_sum += dist;

            values[index] = value;
            dists[index] = dist;
            index ++;
        }
    }

    float result = 0; //
    //float v = img.ptr<float>(row)[col];

    for(int i = 0; i<index; i++)
    {
        if(values[i] == 0)
            continue;

        float rat = dists[i]/dist_sum;
        result += rat*values[i];

//        if(v != 0)
//            cout << "rat: " << rat << "  value: " << values[i] << "  dist: " << dists[i] << "  sum: " << dist_sum<<endl;
    }
//    if(v != 0)
//        cout << result << "  ori: " << v << endl;

    delete [] values;
    delete [] dists;

    return result;
}

Mat Cloud_Image_Convertor::interpolate_image(Mat img)
{
    Mat img_changed = img.clone();

    for(int i = 0; i<m_image_rows; i++) // y
    for(int j = 0; j<m_image_cols; j++) // x
    {
        float value = get_interpolated_value(img, i, j, 5);
        img_changed.ptr<float>(i)[j] = value;
    }

    return img_changed;
}

pcl::PointCloud<pcl::PointXYZRGB> Cloud_Image_Convertor::resampleCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, float robot_x, float robot_y)
{
    float normal_scale;
    init_img(m_image_rows, m_image_cols, 0);

    pcl::PointCloud<pcl::PointXYZRGB> scaled_cloud = scale_cloud(cloud, robot_x, robot_y);
    cout << "finish scale " << endl;

    Mat index_max = set_pix_Value(scaled_cloud);
    cout << "finish set value " << endl;


    ////////////////////////  close operation  //////////////////////////////////////////
    Mat closed, closed_keep, closed_ref, delite, mask_closed, mask_closed_keep, mask_closed8;
    //medianBlur ( m_img_min, dedian, 3);
    Mat element = getStructuringElement(MORPH_RECT, Size(21,21));
    Mat element3 = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat element2 = getStructuringElement(MORPH_RECT, Size(5,5));

//    morphologyEx(m_img_min, closed_keep, MORPH_DILATE, element3);
//    imshow("MORPH_DILATE", closed_keep);
//    morphologyEx(m_img_min, closed, MORPH_CLOSE, element3);
//    imshow("MORPH_CLOSE 1", closed);

    closed = interpolate_image(m_img_min);
    closed = interpolate_image(closed);
    Mat a = normalizeMat(closed);
    Mat b = normalizeMat(m_img_min);
    imshow("closed", a);
    imshow("m_img_min", b);
   // waitKey(30);

    m_img_closed = closed;

    /////////////////////////////  sobel edge  ///////////////////////////////////////////
    Mat img_sobel = normalizeMat(closed, 255, &normal_scale);
    //Mat img_sobel = closed;
    cout << "scale: " << normal_scale << endl;
    Mat dst_x, dst_y, dst;
    Sobel(img_sobel, dst_x, m_img_min.depth(), 1, 0);
    Sobel(img_sobel, dst_y, m_img_min.depth(), 0, 1);
    convertScaleAbs(dst_x, dst_x);
    convertScaleAbs(dst_y, dst_y);
    addWeighted( dst_x, 0.5, dst_y, 0.5, 0, dst);


    //////////////////////// labelling point cloud ////////////////////////////////////////
    double s = 9*m_pix_size/normal_scale;
    dst.convertTo(m_img_gra, CV_32FC1, s, 0);

    //m_img_gra = remove_edgeSobel(m_img_gra);

    // Mat element_3 = getStructuringElement(MORPH_RECT, Size(3, 3));
    // morphologyEx(m_img_gra, m_img_gra, MORPH_DILATE, element_3);

    Mat close_erode;
    Mat close_t;
    threshold(closed, close_t, 0, 255, THRESH_BINARY);
    morphologyEx(close_t, close_erode, MORPH_ERODE, element2);

    m_img_costmap = generate_costmap(close_erode, m_img_gra, m_img_obs, 0.06, 0.4);
    m_costmap_points = getCloutCostMap(m_img_costmap, m_img_closed, robot_x, robot_y);

    //cloud = label_cloud(cloud, close_erode, m_img_gra, index_max, 0.1, 0.4);
    cloud = label_cloud(cloud, m_img_costmap, index_max);
    cout << cloud.points.size()<<endl;
    /////////////////////// display ////////////////////////////////////////////////
    Mat ori = normalizeMat(m_img_min);
    Mat img_closed = normalizeMat(closed);
    Mat img_dilate = normalizeMat(delite);
    m_img_closed = img_closed;


    //imshow("m_img_min", ori);
    imshow("close_erode", close_erode);
    imshow("sobel", dst);
    imshow("costmap", m_img_costmap);
    waitKey(0);

    //morphologyEx(m_img_min, closed, MORPH_ERODE, element);
    //return cloud + m_obs_points;
    return cloud;
}


pcl::PointCloud<pcl::PointXYZRGB> Cloud_Image_Convertor::obstaclePointsDetection_singlescene(pcl::PointCloud<pcl::PointXYZRGB> cloud, float robot_x, float robot_y)
{
    pcl::PointCloud<pcl::PointXYZRGB> non_obspoints;
    non_obspoints.header = cloud.header;

    init_img(m_image_rows, m_image_cols, 0);
    m_obs_points.points.clear();

    pcl::PointCloud<pcl::PointXYZRGB> scaled_cloud = scale_cloud(cloud, robot_x, robot_y);
    cout << "finish scale " << m_min_z << endl;

    Mat index_max = set_pix_Value(scaled_cloud);
    cout << "finish set value " << endl;

    //Mat diff_img = m_img_max - m_img_min;

    Mat delite, delite_min, delite_max;
    Mat element2 = getStructuringElement(MORPH_RECT, Size(5,5));

    morphologyEx(m_img_min, delite_min, MORPH_DILATE, element2);
    morphologyEx(m_img_max, delite_max, MORPH_DILATE, element2);

    Mat diff_img = m_img_max - m_img_min;

    morphologyEx(diff_img, delite, MORPH_DILATE, element2);
    Mat diff = normalizeMat(delite);
    imshow("diff_img", diff);
    waitKey(10);

    for(int i = 0; i<cloud.points.size(); i++)
    {
        int pix_row = index_max.ptr<float>(i)[0];
        int pix_col = index_max.ptr<float>(i)[1];

        float deff_v = delite.ptr<float>(pix_row)[pix_col];

        if(deff_v > 0.12) // obs
        {
            cloud.points[i] = set_point_color(cloud.points[i], 255, 0, 0);
        }
        else
        {
            cloud.points[i] = set_point_color(cloud.points[i], 0, 255, 0); // edge
        }

    }

    return cloud;
}


pcl::PointCloud<pcl::PointXYZRGB> Cloud_Image_Convertor::obstaclePointsDetection(pcl::PointCloud<pcl::PointXYZRGB> cloud, float robot_x, float robot_y, pcl::PointCloud<pcl::Normal> normal)
{
    pcl::PointCloud<pcl::PointXYZRGB> non_obspoints;
    non_obspoints.header = cloud.header;

    init_img(m_image_rows, m_image_cols, 0);

    m_img_obs = Mat(m_image_rows,m_image_cols, CV_8UC1,  Scalar(0)); // init the obstacle map for the pre detection

    m_obs_points.points.clear();

    pcl::PointCloud<pcl::PointXYZRGB> scaled_cloud = scale_cloud(cloud, robot_x, robot_y);
    cout << "finish scale " << m_min_z << endl;

    Mat index_max = set_pix_Value(scaled_cloud);
    cout << "finish set value " << endl;


    bool use_normal = false;
    if(normal.points.size() == cloud.points.size())
        use_normal = true;

    cout << "..........finish set value" << cloud.points.size() << endl;

    for(int i = 0; i<cloud.points.size(); i++)
    {
        if(scaled_cloud.points[i].x == 0 && scaled_cloud.points[i].y == 0)
            continue;
                    
        int pix_row = index_max.ptr<float>(i)[0];
        int pix_col = index_max.ptr<float>(i)[1];

        float height = cloud.points[i].z - m_min_z + 0.7;
       // cout << pix_row << " " << pix_col << " " << height <<  " min_h" << m_min_z << endl;
        
        int type = is_blocked(m_img_allheight, pix_row, pix_col, height, 0);
       // cout << "type" << endl;
        if(type == 2) // obs
        {
            cloud.points[i] = set_point_color(cloud.points[i], 255, 0, 0);
            m_obs_points.points.push_back(cloud.points[i]);
            m_img_obs.ptr<uchar>(pix_row)[pix_col] = 3;

        }
        else if(type == 1 )
        {
            cloud.points[i] = set_point_color(cloud.points[i], 0, 255, 0); // shink
            non_obspoints.points.push_back(cloud.points[i]);
            m_img_obs.ptr<uchar>(pix_row)[pix_col] = 2;
        }
        else if(type == 0)
        {
            cloud.points[i] = set_point_color(cloud.points[i], 0, 0, 255); // walk blue
            non_obspoints.points.push_back(cloud.points[i]);
            m_img_obs.ptr<uchar>(pix_row)[pix_col] = 1;
        }
       //         cout << "f" << endl;
    }

    m_all_points = cloud;
    return non_obspoints;
    //return cloud;
}