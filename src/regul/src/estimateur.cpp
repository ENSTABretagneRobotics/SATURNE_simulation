#include "ros/ros.h"
#include "tf/tf.h"
#include <cmath>
#include "UTM.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "regul/msg_estim.h"



float x_pos,y_pos;
double current_heading;
double current_vel;


bool incorrect_gps = false;
bool ready = false;

double max_cov = 5.;

ros::Time last_msg_imu;
ros::Time last_msg_gps;
ros::Time last_msg_vel;

void callbackFix(const sensor_msgs::NavSatFix& msg)
{
	double current_lat = msg.latitude;
	double current_long = msg.longitude;
	incorrect_gps = false;

	if(msg.position_covariance[0] > max_cov || msg.position_covariance[4] > max_cov || (isnan(current_lat) || isnan(current_long)))
	{
		incorrect_gps = true;
	}

	if(incorrect_gps == false)
	{
		LatLonToUTMXY(current_lat,current_long,0,y_pos,x_pos);
		ready = true;
	}
	else
	{
		ROS_WARN("Les donnees du GPS ne sont pas assez precises.");
	}
	last_msg_gps = ros::Time::now();
}

void callbackImu(const sensor_msgs::Imu& msg)
{
	current_heading = tf::getYaw(msg.orientation);
	last_msg_imu = ros::Time::now();
}

void callbackVel(const geometry_msgs::TwistStamped& msg)
{
	double vx = msg.twist.linear.x;
	double vy = msg.twist.linear.y;
	if(isnan(vx) || isnan(vy))
	{
		current_vel = 0;
	}
	else
	{
		current_vel = sqrt(vx*vx+vy*vy);
	}
	last_msg_vel = ros::Time::now();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Estim_node");

	ros::NodeHandle n;

	ros::Subscriber sub_fix = n.subscribe("/fix", 1000, callbackFix);
	ros::Subscriber sub_vel = n.subscribe("/vel", 1000, callbackVel);
	ros::Subscriber sub_imu = n.subscribe("/imu/data", 1000, callbackImu);
 	ros::Publisher chatter_estim = n.advertise<regul::msg_estim>("/estim", 1000);

  	ros::Rate loop_rate(10);
 	regul::msg_estim msg;


	ros::Time loop_time = ros::Time::now();
    while (ros::ok())
    {
    	if((ros::Time::now()-last_msg_vel).toSec() <= 2.)//?
    	{
    		double dt = (loop_time - ros::Time::now()).toSec();
    		loop_time = ros::Time::now();
    		x_pos -= current_vel*dt*cos(-current_heading);
    		y_pos += current_vel*dt*sin(-current_heading);
    	}
    	msg.x = x_pos;
    	msg.y = y_pos;
    	msg.theta = current_heading;
    	msg.vel = current_vel;
    	//msg.correct = true; // For SATURNE simulator...
    	msg.correct = (ready && incorrect_gps == false && (ros::Time::now()-last_msg_imu).toSec() <= 2. && (ros::Time::now()-last_msg_gps).toSec() <= 2. && (ros::Time::now()-last_msg_vel).toSec() <= 2.); // For SATURNE or Warthog...
		chatter_estim.publish(msg);
        loop_rate.sleep();
	    ros::spinOnce();
    }
	return 0;
}