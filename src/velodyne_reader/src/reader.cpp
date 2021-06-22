#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "regul/msg_obj.h"


#include <limits>
#include <math.h>
#include <algorithm>

#include <Eigen/Dense>
#include <vector>

#define infinity std::numeric_limits<float>::infinity()  

// Securiser le cas des matrices vides...
float dist_obj = 2.;
ros::Publisher chatter_obj;
ros::Publisher chatter_emergency;

float estim_theta;
bool callEmergency = false;

std::pair<double,double> leastSquares(std::vector<float> x_vect, std::vector<float> y_vect)
{
	if(x_vect.size() != 0 and y_vect.size() != 0)
	{
		Eigen::VectorXf b  = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(y_vect.data(), y_vect.size());
		Eigen::MatrixXf A(x_vect.size(),2);
		A << Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(x_vect.data(), x_vect.size()),Eigen::VectorXf::Ones(x_vect.size());
		Eigen::Vector2f p = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
		return std::make_pair(p(0),p(1));
	}
	else
	{
		//ROS_ERROR("Données incorrectes");
		return std::make_pair(-1000.,-1000.);
	}

}	


void cmdCallback(const sensor_msgs::PointCloud2& msg)
{
	sensor_msgs::PointCloud out_cloud;
	sensor_msgs::convertPointCloud2ToPointCloud(msg, out_cloud);
	//long n = 0;
	//float dist_mean = 0;
	std::vector<float> x_vect_left;
	std::vector<float> y_vect_left;
	std::vector<float> x_vect_right;
	std::vector<float> y_vect_right;

	float angle_max = 60.;
	float angle_min = 0.;


	int count = 0;
	for(int i = 0 ; i < out_cloud.points.size(); ++i)
	{
		float x = out_cloud.points[i].x;
	    float y = out_cloud.points[i].y;
	    float z = out_cloud.points[i].z;
	    float dist = std::sqrt(x*x+y*y);
	    float angle = (atan2(y,x)*180./M_PI);
	    if(dist < 15.)
	    {
	        if(z > -0.6 and z <= 0.10 and angle > angle_min and angle < angle_max)
		    {		    
		            x_vect_left.push_back(x);
		            y_vect_left.push_back(y);
	    	}
	        if(z > -0.6 and z <= 0.10 and angle < -angle_min and angle > -angle_max)
		    {		    
		            x_vect_right.push_back(x);
		            y_vect_right.push_back(y);
	    	}
	    	if(angle > -20. and angle < 20. and dist < 1.5)
	    	{
	    		count++;
	    	}
    	}
	}
	std::pair<double,double> p_right = leastSquares(x_vect_right,y_vect_right);
	std::pair<double,double> p_left = leastSquares(x_vect_left,y_vect_left);
	std::cout <<"Right -> a : " <<p_right.first <<" b : "<< p_right.second<<  std::endl;
	std::cout <<"Left  -> a : " <<p_left.first <<" b : "<< p_left.second<<  std::endl;
	std::cout << std::endl;
/*
	std::vector<float> dist_right_vect;
	std::vector<float> dist_left_vect;

	float ax_right,ay_right,bx_right,by_right;
	ax_right = 0.;
	bx_right = 5.;
	ay_right = p_right.first*ax_right + p_right.second;
	by_right = p_right.first*bx_right + p_right.second;

	float ax_left,ay_left,bx_left,by_left;
	ax_left = 0.;
	bx_left = 5.;
	ay_left = p_left.first*ax_left + p_left.second;
	by_left = p_left.first*bx_left + p_left.second;

	for(int i = 0; i < x_vect_right.size(); i++)
	{
		float norm = sqrt((bx_right - ax_right)*(bx_right - ax_right)+(by_right - ay_right)*(by_right - ay_right));
		float det_mat_dist = (bx_right - ax_right)*(y_vect_right[i] - ay_right) - (by_right - ay_right) * (x_vect_right[i] - ax_right);
		float dist_ligne = det_mat_dist/norm;
		dist_right_vect.push_back(fabs(dist_ligne));
	}

	for(int i = 0; i < x_vect_left.size(); i++)
	{
		float norm = sqrt((bx_left - ax_left)*(bx_left - ax_left)+(by_left - ay_left)*(by_left - ay_left));
		float det_mat_dist = (bx_left - ax_left)*(y_vect_left[i] - ay_left) - (by_left - ay_left) * (x_vect_left[i] - ax_left);
		float dist_ligne = det_mat_dist/norm;
		dist_left_vect.push_back(fabs(dist_ligne));
	}

  	std::sort(dist_right_vect.begin(), dist_right_vect.end());
  	std::sort(dist_left_vect.begin(), dist_left_vect.end());
  	int pos_right = (int)(x_vect_right.size()*0.5);
  	int pos_left = (int)(x_vect_left.size()*0.5);

  	float coeff_a_right = p_right.first;
  	float coeff_b_right = fabs(p_right.second);
  	if(dist_right_vect[pos_right] > 1.)
  	{
  		coeff_b_right -= dist_right_vect[pos_right]; 
  	}
  	
    if(dist_left_vect[pos_left] > 1.)
  	{
  		dist_obj_act -=  dist_left_vect[pos_left];
  	}	
*/
	float coeff_a_right = p_right.first;
  	float coeff_b_right = fabs(p_right.second);

  	float coeff_a_left = p_left.first;
  	float coeff_b_left = fabs(p_left.second);

	float ctrl_dist = dist_obj - coeff_b_right;

  	if(coeff_b_left < 1.5)
  	{
  		ctrl_dist = (coeff_b_right+coeff_b_left)/2. - coeff_b_right;
  	}

	if(p_right.second != -1000.)
	{
		regul::msg_obj msg_;
		msg_.heading = -2.*(ctrl_dist) - 2.*coeff_a_right;
		if(msg_.heading > M_PI/4.)
		{
			msg_.heading = M_PI/4.;
		}
		if(msg_.heading < -M_PI/4.)
		{
			msg_.heading = -M_PI/4.;
		}
		//std::cout << dist_left_vect[pos_left] <<" " << dist_right_vect[pos_right] <<" "  <<msg_.heading*180./M_PI <<std::endl; 
		msg_.mode_heading = false;
	    chatter_obj.publish(msg_);
	    

	}
	std_msgs::Bool msg_em;
    if(count >= 20)
    {
    	msg_em.data = true;
    	chatter_emergency.publish(msg_em);
    	callEmergency = true;
    }
    else if(callEmergency)
    {
    	msg_em.data = false;
    	chatter_emergency.publish(msg_em);
    	callEmergency = false;
    }

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Reader_node");

	ros::NodeHandle n;
	ros::Subscriber sub_cloud = n.subscribe("/velodyne_points", 1000, cmdCallback);
	chatter_obj = n.advertise<regul::msg_obj>("/obj", 1000);
	chatter_emergency = n.advertise<std_msgs::Bool>("/emergency", 1000);
	ros::Rate loop_rate(10);
	ros::spin();

	return 0;
}