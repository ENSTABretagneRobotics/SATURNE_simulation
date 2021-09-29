#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"

geometry_msgs::Twist twist_sub_msg;

void twist_sub_cb(const geometry_msgs::Twist::ConstPtr& twist)
{
	twist_sub_msg = *twist;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "twist_conv");
	ros::NodeHandle nh("~");

	ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>(nh.param<std::string>("subscriber/topic", "/twist"), nh.param("subscriber/queue", 1000), twist_sub_cb);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(nh.param<std::string>("publisher/topic", "/cmd_vel"), nh.param("publisher/queue", 1000));

	ros::Rate rate(nh.param("rate", 50));

	while (ros::ok()) {
		geometry_msgs::Twist twist_pub_msg;

		twist_pub_msg = twist_sub_msg;
		twist_pub_msg.angular.x = nh.param("angular_x_coef", 1.0)*twist_sub_msg.angular.x;
		twist_pub_msg.angular.y = nh.param("angular_y_coef", 1.0)*twist_sub_msg.angular.y;
		twist_pub_msg.angular.z = nh.param("angular_z_coef", 1.0)*twist_sub_msg.angular.z;
		twist_pub_msg.linear.x = nh.param("linear_x_coef", 1.0)*twist_sub_msg.linear.x;
		twist_pub_msg.linear.y = nh.param("linear_y_coef", 1.0)*twist_sub_msg.linear.y;
		twist_pub_msg.linear.z = nh.param("linear_z_coef", 1.0)*twist_sub_msg.linear.z;

		pub.publish(twist_pub_msg);

		rate.sleep();

		ros::spinOnce();
	}
}
