#ifndef _XI_USEFUL_FUNCTIONS_H_  
#define _XI_USEFUL_FUNCTIONS_H_  

float toDegree = 180/M_PI;

float get_e_distance_2d(float x1, float y1, float x2, float y2)
{
	float diff_x = abs(x1 - x2);
	float diff_y = abs(y1 - y2);
	
	float distance = sqrt(diff_x * diff_x + diff_y * diff_y);
	
	return distance;
}

void get_normal_from_two_points(float x1, float y1, float x2, float y2, float &normal_x, float &normal_y)
{
	float x = x2 - x1;
	float y = y2 - y1;
	
	float slope = x/y;
	float squre = 1/(1+slope*slope);
	
	normal_y = sqrt(squre);
	normal_x = -(normal_y / slope);
}

float get_angle_from_twopoints(float x1, float y1, float x2, float y2)
{
	float x = x2 - x1;
	float y = y2 - y1;
	
	float angle = atan2(y, x) * toDegree;
	
	return angle;
}

float normalize_angle_by_dist(float angle_1, float angle_2, float dist_1, float dist_2)
{
	float sum_dist = dist_1 + dist_2;
	float angle = angle_1 * dist_2/sum_dist + angle_2 * dist_1/sum_dist;
	
	return angle;		
}

float get_normalized_angle(float x1, float y1, float x2, float y2, float x3, float y3)
{
	float angle_1 = get_angle_from_twopoints(x1, y1, x2, y2);
	float angle_2 = get_angle_from_twopoints(x2, y2, x3, y3);
	
	float dist_1 = get_e_distance_2d(x1, y1, x2, y2);
	float dist_2 = get_e_distance_2d(x2, y2, x3, y3);
	
	float angle = normalize_angle_by_dist(angle_1, angle_2, dist_1, dist_2);
	
	return angle;		
}

// void normalize_by_distance( float normal_x_1, float normal_y_1, 
// 							float normal_x_2, float normal_y_2, 
// 							float dist_1, float dist_2
// 							float &normal_x, float &normal_y)
// {
// 	float sum = dist_1 + dist_2;
	
// }

// void get_normalized_normal( float x1, float y1,
// 						    float x2, float y2,
// 							float x3, float y3,
// 							float &normal_x, float &normal_y)
// {
// 	float normal_x_L, normal_y_L, normal_x_R, normal_y_R;
	
// 	float dist_1 = get_e_distance_2d(x1, y1, x2, y2);
// 	float dist_2 = get_e_distance_2d(x2, y2, x3, y3);
	
// 	get_normal_from_two_points(x2, y2, x1, y1, normal_x_L, normal_y_L);
// 	get_normal_from_two_points(x2, y2, x3, y3, normal_x_R, normal_y_R);
	
	
// }

#endif