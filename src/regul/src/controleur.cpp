#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "roboteq/msg_cmd_esc.h"
#include <cmath>
#include "regul/msg_estim.h"
#include "regul/msg_obj.h"

ros::Publisher pub_cap;

float estim_x,estim_y;
double estim_theta,estim_vel;
bool estim_correct;

float obj_x,obj_y,obj_heading;
float var_heading;
bool mode_heading;

//int max_speed = 200; //500
//int max_var_speed = 150; //200
double max_angular_z = 0.7;
//int speed = 200;
ros::Time last_msg_estim;
ros::Time last_msg_obj;
bool emergency_mode = false;

//int speed_check(int spd)
//{
//	return std::min(std::max(spd,max_speed-max_var_speed),max_speed+max_var_speed);
//}

void callbackEstim(const regul::msg_estim& msg)
{
	estim_x = msg.x;
	estim_y = msg.y;
	estim_theta = msg.theta;
	estim_correct = msg.correct;
	estim_vel = msg.vel;
	last_msg_estim = ros::Time::now();
}

void callbackObj(const regul::msg_obj& msg)
{
	mode_heading = msg.mode_heading;
	if(mode_heading)
	{
		obj_heading = msg.heading;
	}
	else
	{
		var_heading = msg.heading;
	}
	last_msg_obj = ros::Time::now();
}

void callbackEmergency(const std_msgs::Bool::ConstPtr& msg)
{
	emergency_mode = msg->data;
}

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Ctrl_node");

	ros::NodeHandle n;
	ros::Subscriber sub_estim = n.subscribe("/estim", 1000, callbackEstim);
	ros::Subscriber sub_obj = n.subscribe("/obj", 1000, callbackObj);
	ros::Subscriber sub_emergency = n.subscribe("/emergency", 1000, callbackEmergency);
 	//ros::Publisher chatter_cmd_front = n.advertise<roboteq::msg_cmd_esc>("/front/cmd_esc", 1000);
	//ros::Publisher chatter_cmd_back = n.advertise<roboteq::msg_cmd_esc>("/back/cmd_esc", 1000);
	ros::Publisher chatter_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  	ros::Rate loop_rate(10);

    while (ros::ok())
    {
		//roboteq::msg_cmd_esc msg;
		geometry_msgs::Twist msg_cmd_vel;
		double current_objective_heading;
	    //double diff;
	    if(!mode_heading)
	    {
        	//speed = 300;
	    	//diff = 300.*sawtooth(var_heading);
	    	//msg.cmd_left = speed_check((int)(speed+diff));
	    	//msg.cmd_right = speed_check((int)(speed-diff));
			msg_cmd_vel.angular.x = 0; msg_cmd_vel.angular.y = 0; msg_cmd_vel.angular.z = std::min(std::max(sawtooth(var_heading),-max_angular_z),max_angular_z);
			msg_cmd_vel.linear.x = 0.9; msg_cmd_vel.linear.y = 0; msg_cmd_vel.linear.z = 0;
		}
        else
        {
        	//speed = 300;
        	current_objective_heading = obj_heading;
        	//diff = 300.*sawtooth(current_objective_heading - estim_theta);
        	//msg.cmd_left = speed_check((int)(speed+diff));
	    	//msg.cmd_right = speed_check((int)(speed-diff));
			msg_cmd_vel.angular.x = 0; msg_cmd_vel.angular.y = 0; msg_cmd_vel.angular.z = std::min(std::max(sawtooth(estim_theta-current_objective_heading),-max_angular_z),max_angular_z);
			msg_cmd_vel.linear.x = 0.9; msg_cmd_vel.linear.y = 0; msg_cmd_vel.linear.z = 0;
			if(estim_correct == false || (ros::Time::now()-last_msg_obj).toSec() > 1. || (ros::Time::now()-last_msg_estim).toSec() > 1.)
			{
				 //msg.cmd_left = 0;
				 //msg.cmd_right = 0;
				 msg_cmd_vel.angular.x = 0; msg_cmd_vel.angular.y = 0; msg_cmd_vel.angular.z = 0;
				 msg_cmd_vel.linear.x = 0; msg_cmd_vel.linear.y = 0; msg_cmd_vel.linear.z = 0;
			}
        }
        if(emergency_mode)
        {
        	//msg.cmd_left = 0;
			//msg.cmd_right = 0;	
			msg_cmd_vel.angular.x = 0; msg_cmd_vel.angular.y = 0; msg_cmd_vel.angular.z = 0;
			msg_cmd_vel.linear.x = 0; msg_cmd_vel.linear.y = 0; msg_cmd_vel.linear.z = 0;
		}
        
	    //chatter_cmd_front.publish(msg);
	    //chatter_cmd_back.publish(msg);
		chatter_cmd_vel.publish(msg_cmd_vel);
        loop_rate.sleep();
	    ros::spinOnce();
    }

  
	return 0;
}
