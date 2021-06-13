#include "ros/ros.h"
#include "tf/tf.h"
#include <cmath>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

ros::Publisher chatter_vel;

double dist_movement;
double angle_rotation;
double yaw_obj;

double yaw;
double x_encod;

double epsilon_dist = 0.05;
double epsilon_angle = 0.03;
bool isMoving = false;
bool order_rotation = false;
bool order_received = false;

double yawBeforeMovement;
double posBeforeMovement;

double v_max = 0.5;
double v_min = 0.1;

double v_rot_max = 6.5;//4
double v_rot_min = 1.5;
geometry_msgs::Twist msg_send;


ros::Time last_msg_imu;
ros::Time last_msg_enc;

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}


void callbackImu(const sensor_msgs::Imu& msg)
{
	yaw = tf::getYaw(msg.orientation);
	last_msg_imu = ros::Time::now();
}


void callbackRotation(const std_msgs::Float64& msg)
{
	if(msg.data != 0. and !order_received)
	{
		angle_rotation = -msg.data*M_PI/180.;
		order_received = true;
		order_rotation = true;
	}
}

void callbackStraightLine(const std_msgs::Float64& msg)
{
	if(msg.data != 0. and !order_received)
	{
		dist_movement = msg.data;
		order_received = true;
		order_rotation = false;
	}
}

void callbackEncoders(const geometry_msgs::Twist& msg)
{
	x_encod = msg.linear.x;
	last_msg_enc = ros::Time::now();
}

double sign(double x)
{
	if(x >=0)
	{
		return 1.;
	}
	else
	{
		return -1.;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Encoders_ctrl_node");

	ros::NodeHandle n;
	ros::Subscriber sub_encoders = n.subscribe("/encoders", 1000, callbackEncoders);
	ros::Subscriber sub_imu = n.subscribe("/imu/data", 1000, callbackImu);
	ros::Subscriber sub_order_line = n.subscribe("/cmd_straight_line", 1000, callbackStraightLine);
	ros::Subscriber sub_order_rotation = n.subscribe("/cmd_rotation", 1000, callbackRotation);
 	chatter_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate loop_rate(5);
 	while (ros::ok())
	{
		msg_send.linear.x = 0.;
		msg_send.linear.z = 0.;
		msg_send.angular.z = 0.;

		if(order_received and !isMoving and !order_rotation and (ros::Time::now()-last_msg_enc).toSec() <= 0.5)
		{
			posBeforeMovement = x_encod;
			isMoving = true;
		}
		if(isMoving and !order_rotation and (ros::Time::now()-last_msg_enc).toSec() <= 0.5)
		{
			double delta = (posBeforeMovement + dist_movement) - x_encod;

			if((dist_movement > 0 and delta <= 0 and std::fabs(delta) < epsilon_dist) or (dist_movement < 0 and delta >= 0 and std::fabs(delta) < epsilon_dist))
			{
				ROS_INFO("Done");
				isMoving = false;
				order_received = false;
			    msg_send.linear.x = 0.;
		    	msg_send.angular.z = 0.;
			}
			else
			{
				if(delta > 0)
				{
		    		msg_send.linear.x = std::max(v_max*atan(delta),v_min);
		    	}
		    	else
		    	{
		    		msg_send.linear.x = std::min(v_max*atan(delta),-v_min);
		    	}
		    	msg_send.angular.z = 0.;
		    }
		    ROS_INFO("Linear: %f %f",delta,msg_send.linear.x);
		}

		if(order_received and !isMoving and order_rotation and (ros::Time::now()-last_msg_imu).toSec() <= 0.15)
		{
			yawBeforeMovement = yaw;
			yaw_obj = yawBeforeMovement + angle_rotation;
			isMoving = true;
		}
		if(isMoving and order_rotation and (ros::Time::now()-last_msg_imu).toSec() <= 0.15)
		{
			double delta = sawtooth(yaw_obj - yaw);

			if(std::fabs(delta) < epsilon_angle)
			{
				ROS_INFO("Done");
				isMoving = false;
				order_received = false;
			    msg_send.linear.x = 0.;
		    	msg_send.angular.z = 0.;
			}
			else
			{
				msg_send.linear.x = 0.;
		    	msg_send.angular.z = std::min(std::max(-v_rot_max*atan(delta),-v_rot_min),v_rot_min);
		    }
		    ROS_INFO("Angular: %f %f %f %f",yaw*180/M_PI,delta*180./M_PI,yaw_obj*180./M_PI,msg_send.angular.z);

		}
	    chatter_vel.publish(msg_send);
		loop_rate.sleep();
		ros::spinOnce();
    }


  
	return 0;
}
