#include "ros/ros.h"
#include "tf/tf.h"
#include <cmath>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

double cmd_rc_right = 0.;
double cmd_rc_left = 0.;
double cmd_auto_right = 0;
double cmd_auto_left = 0.;
double last_cmd_right = 0.;
double last_cmd_left = 0.;

bool rc_enabled = false;

ros::Time last_msg_rc_left;
ros::Time last_msg_rc_right;
ros::Time last_msg_rc_enabled;
ros::Time last_msg_cmd_vel;

const double max_accel = 10.;
const double max_velocity = 5.;
const double coeff_sevcon = 10./3.;
const int freq = 15;

const double timeOut_RC = 0.1;
const double timeOut_cmdVel = 0.15;


bool checkTimeRC()
{
	ros::Time current_time = ros::Time::now();
	if((current_time - last_msg_rc_left).toSec() < timeOut_RC and (current_time - last_msg_rc_right).toSec() < timeOut_RC and (current_time - last_msg_rc_enabled).toSec() < timeOut_RC)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool checkTimeCmdVel()
{
	ros::Time current_time = ros::Time::now();
	if((current_time - last_msg_cmd_vel).toSec() < timeOut_cmdVel)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void callbackCmdVel(const geometry_msgs::Twist& msg)
{
	cmd_auto_left = std::min(max_velocity,std::max(-max_velocity,msg.linear.x - msg.angular.z * 1.08/2.)); //Ou 1.21m
	cmd_auto_right = std::min(max_velocity,std::max(-max_velocity,msg.linear.x + msg.angular.z *1.08/2.));
	last_msg_cmd_vel = ros::Time::now();
}

void callbackRCLeft(const std_msgs::Float64& msg)
{
	cmd_rc_left = msg.data*max_velocity/100.;
	last_msg_rc_left = ros::Time::now();
}

void callbackRCRight(const std_msgs::Float64& msg)
{
	cmd_rc_right = msg.data*max_velocity/100.;
	last_msg_rc_right = ros::Time::now();
}

void callbackRCEnabled(const std_msgs::Bool& msg)
{
	rc_enabled = msg.data;
	last_msg_rc_enabled = ros::Time::now();
}

std::pair<double,double> ramp(double cmd_left, double cmd_right)
{
	double output_left,output_right;
	if((cmd_left - last_cmd_left) > 0) //Acceleration
	{
		if((cmd_left - last_cmd_left)*(double)freq > max_accel)
		{
			output_left = last_cmd_left + max_accel/(double)freq;
		}
		else
		{
			output_left = cmd_left;
		}
	}
	else //Deceleration
	{
	    if((cmd_left - last_cmd_left)*(double)freq < -max_accel)
		{
			output_left = last_cmd_left - max_accel/(double)freq;
		}
		else
		{
			output_left = cmd_left;
		}	
	}

	if((cmd_right - last_cmd_right) > 0) //Acceleration
	{
		if((cmd_right - last_cmd_right)*(double)freq > max_accel)
		{
			output_right = last_cmd_right + max_accel/(double)freq;
		}
		else
		{
			output_right = cmd_right;
		}
	}
	else //Deceleration
	{
	    if((cmd_right - last_cmd_right)*(double)freq < -max_accel)
		{
			output_right = last_cmd_right - max_accel/(double)freq;
		}
		else
		{
			output_right = cmd_right;
		}	
	}
	return std::make_pair(output_left,output_right);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Ctrl_node");

	ros::NodeHandle n;
	ros::Subscriber sub_rcleft = n.subscribe("/rc/left", 1000, callbackRCLeft);
	ros::Subscriber sub_rcright = n.subscribe("/rc/right", 1000, callbackRCRight);
	ros::Subscriber sub_rcenabled = n.subscribe("/rc/enabled", 1000, callbackRCEnabled);
	ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel", 1000, callbackCmdVel);

 	ros::Publisher chatter_vel_left = n.advertise<std_msgs::Float64>("/left_drive/velocity", 1000);
 	ros::Publisher chatter_vel_right = n.advertise<std_msgs::Float64>("/right_drive/velocity", 1000);

  	ros::Rate loop_rate(freq);
 
    while (ros::ok())
    {
    	std_msgs::Float64 msg_left,msg_right;
    	std::pair<double,double> output = {0.,0.};
    	if(checkTimeRC() == true and rc_enabled == true)
    	{
    		output = ramp(cmd_rc_left,cmd_rc_right);
    	}
    	else if(checkTimeCmdVel() == true)
    	{
    		output = ramp(cmd_auto_left,cmd_auto_right);
    	}
    	
		last_cmd_right = output.second;
		last_cmd_left = output.first;

		msg_right.data = output.second*coeff_sevcon;
		msg_left.data = output.first*coeff_sevcon;

	    
	    chatter_vel_left.publish(msg_left);
	    chatter_vel_right.publish(msg_right);
        loop_rate.sleep();
	    ros::spinOnce();
    }

  
	return 0;
}
