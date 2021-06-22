#include "ros/ros.h"
#include <cmath>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

double pos = 0.;
double angle = 0.;

double wheel_separation = 1.21;
double last_enc_right = 0.;
double last_enc_left = 0.;
double wheel_radius = 0.30;
double wheel_to_track = 0.5962;


double lastSpdRight = 0.;
double lastSpdLeft = 0.;
double lastDtRight;
double lastDtLeft;
bool unUsedSpdLeft = false;
bool unUsedSpdRight = false;

ros::Time last_msg_enc_right;
ros::Time last_msg_enc_left;
ros::Publisher chatter_cmd;

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}

void callbackEncRight(const std_msgs::Float64 &msg)
{
	double dt = (ros::Time::now() - last_msg_enc_right).toSec();
	double v_right = 0.5*wheel_radius*(msg.data+last_enc_right); //Intergration trapèze
	
	lastSpdRight = v_right;
	lastDtRight = dt;
	unUsedSpdRight = true;

	last_enc_right = msg.data;
	last_msg_enc_right = ros::Time::now();

}

void callbackEncLeft(const std_msgs::Float64 &msg)
{
	double dt = (ros::Time::now() - last_msg_enc_left).toSec();
	double v_left = 0.5*wheel_radius*(msg.data+last_enc_left); //Intergration trapèze
	lastSpdLeft = v_left;
	lastDtLeft = dt;
	unUsedSpdLeft = true;

	last_enc_left = msg.data;
	last_msg_enc_left = ros::Time::now();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Encoders_node");

	ros::NodeHandle n;
	ros::Subscriber sub_enc_right = n.subscribe("/right_drive/status/speed", 1000, callbackEncRight);
	ros::Subscriber sub_enc_left = n.subscribe("/left_drive/status/speed", 1000, callbackEncLeft);
	chatter_cmd = n.advertise<geometry_msgs::Twist>("/encoders", 1000);
	last_msg_enc_right = ros::Time::now();
	last_msg_enc_left = ros::Time::now();
	ros::Publisher chatter_vel = n.advertise<nav_msgs::Odometry>("/odom", 1000);
	ros::Rate loop_rate(20);
	nav_msgs::Odometry msg;
	geometry_msgs::Twist msg_pos;
 	while(ros::ok())
 	{
 		if(unUsedSpdLeft and unUsedSpdRight)
 		{
 			unUsedSpdRight = false;
 			unUsedSpdLeft = false;

 			double dx_right = lastSpdRight*lastDtRight;
 			double dx_left = lastSpdLeft*lastDtLeft;
 			//std::cout << lastSpdRight << " " << lastDtRight << std::endl;
 			double spd = (lastSpdRight + lastSpdLeft)/2.;
 			double spd_ang = (lastSpdRight - lastSpdLeft)/wheel_separation;
 			
 			pos += (dx_right + dx_left)/2.;
 			angle = sawtooth(angle + 0.5*(dx_right - dx_left)/wheel_separation);


			ROS_INFO("Pos: %f angle: %f Spd: %f Spd_ang : %f Dx left: %f Dx right: %f",pos,angle*180./M_PI,spd,spd_ang*180./M_PI,dx_left,dx_right);


			msg_pos.linear.x = pos;
			msg_pos.angular.z = angle;
			
 			msg.header.stamp = ros::Time::now();
 			msg.header.frame_id = "odom";
 			msg.twist.twist.linear.x = spd;
 			msg.twist.twist.linear.y = 0.;
 			msg.twist.twist.angular.z = spd_ang;
 			chatter_cmd.publish(msg_pos);
 			chatter_vel.publish(msg);
 		}
        loop_rate.sleep();
	    ros::spinOnce();
 	}

  
	return 0;
}
