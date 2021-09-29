#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "tf/tf.h"

sensor_msgs::Imu imu_sub_msg;

void imu_sub_cb(const sensor_msgs::Imu::ConstPtr& imu)
{
	imu_sub_msg = *imu;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "imu_conv");
	ros::NodeHandle nh("~");

	ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>(nh.param<std::string>("subscriber/topic", "/imu"), nh.param("subscriber/queue", 1000), imu_sub_cb);
	ros::Publisher pub = nh.advertise<sensor_msgs::Imu>(nh.param<std::string>("publisher/topic", "/imu/data"), nh.param("publisher/queue", 1000));

	ros::Rate rate(nh.param("rate", 50));

	while (ros::ok()) {
		sensor_msgs::Imu imu_pub_msg;

		imu_pub_msg = imu_sub_msg;
		imu_pub_msg.angular_velocity.x = nh.param("angular_velocity_x_coef", 1.0)*imu_sub_msg.angular_velocity.x;
		imu_pub_msg.angular_velocity.y = nh.param("angular_velocity_y_coef", 1.0)*imu_sub_msg.angular_velocity.y;
		imu_pub_msg.angular_velocity.z = nh.param("angular_velocity_z_coef", 1.0)*imu_sub_msg.angular_velocity.z;
		imu_pub_msg.linear_acceleration.x = nh.param("linear_acceleration_x_coef", 1.0)*imu_sub_msg.linear_acceleration.x;
		imu_pub_msg.linear_acceleration.y = nh.param("linear_acceleration_y_coef", 1.0)*imu_sub_msg.linear_acceleration.y;
		imu_pub_msg.linear_acceleration.z = nh.param("linear_acceleration_z_coef", 1.0)*imu_sub_msg.linear_acceleration.z;
		tf::Quaternion q;
		double roll = 0, pitch = 0, yaw = 0;
		tf::quaternionMsgToTF(imu_sub_msg.orientation, q);
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		q.setRPY(nh.param("roll_coef", 1.0)*roll, nh.param("pitch_coef", 1.0)*pitch, nh.param("yaw_coef", 1.0)*yaw);
		imu_pub_msg.orientation.x = q[0];
		imu_pub_msg.orientation.y = q[1];
		imu_pub_msg.orientation.z = q[2];
		imu_pub_msg.orientation.w = q[3];

		pub.publish(imu_pub_msg);

		rate.sleep();

		ros::spinOnce();
	}
}
