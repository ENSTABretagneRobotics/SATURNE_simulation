#include "ros/ros.h"
#include "tf/tf.h"
#include <cmath>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"

double yaw = 0.;

double speed_objective = 0.;
double yaw_objective = 0.;

ros::Time last_msg_imu;
ros::Time last_msg_cmd;

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}

void callbackImu(const sensor_msgs::Imu& msg)
{
	yaw = tf::getYaw(msg.orientation);
	last_msg_imu = ros::Time::now();
}

void callbackCmd(const geometry_msgs::Twist& msg)
{
	speed_objective = msg.linear.x;
	yaw_objective = msg.angular.z;
	last_msg_cmd = ros::Time::now();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Ctrl_node");

	ros::NodeHandle n;
	ros::Subscriber sub_imu = n.subscribe("/imu/data", 1000, callbackImu);
	ros::Subscriber sub_objspd = n.subscribe("/cmd_vel_yaw", 1000, callbackCmd);
 	ros::Publisher chatter_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
 	ros::Publisher chatter_deg = n.advertise<std_msgs::Float64>("/yaw_deg", 1000);


  	ros::Rate loop_rate(10);
 
	while (ros::ok())
	{
    	geometry_msgs::Twist msg;
    	std_msgs::Float64 msg_deg;
	    if((ros::Time::now()-last_msg_imu).toSec() <= 0.15 and (ros::Time::now()-last_msg_cmd).toSec() <= 0.25)
	    {
	    	msg.linear.x = speed_objective;
	    	msg.angular.z = -2.5*atan(sawtooth(yaw_objective - yaw));
    	}
	    else
	    {
	    	msg.linear.x = 0.;
	    	msg.angular.z = 0.;
	    }
    msg_deg.data = yaw*180./M_PI;
    chatter_vel.publish(msg);
    chatter_deg.publish(msg_deg);
    loop_rate.sleep();
    ros::spinOnce();
    }

  
	return 0;
}
