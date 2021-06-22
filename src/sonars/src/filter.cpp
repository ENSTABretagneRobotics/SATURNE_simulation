#include "ros/ros.h"
#include "tf/tf.h"
#include <cmath>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

ros::Publisher chatter_sonar_front;
ros::Publisher chatter_sonar_right;
ros::Publisher chatter_sonar_left;
 
std::vector<double> data_front_right;
std::vector<double> data_front_left;
std::vector<double> data_front;
std::vector<double> data_right;
std::vector<double> data_left;


int order = 5;
int order_median = 2;


double addValueAndGetMedian(std::vector<double> &dist_list,double newValue)
{
	dist_list.push_back(newValue);
	if(dist_list.size() > order)
	{
		dist_list.erase(dist_list.begin());
	}
	
	if(dist_list.size() == order)
	{
		std::vector<double> dist_list_sorted;
		for(int i = 0; i < order; i++)
		{
			dist_list_sorted.push_back(dist_list[i]);
		}
		std::sort(dist_list_sorted.begin(), dist_list_sorted.end());
		return dist_list_sorted.at(order_median);
	}
	else
	{
		return -1.;
	}
}

void callbackPololu(const std_msgs::Float64MultiArray msg)
{
	std_msgs::Float64 msg_to_send;
	msg_to_send.data = addValueAndGetMedian(data_left,msg.data[1]);
	chatter_sonar_left.publish(msg_to_send);
	msg_to_send.data = addValueAndGetMedian(data_right,msg.data[0]);
	chatter_sonar_right.publish(msg_to_send);
	double min_data = std::min(std::min(addValueAndGetMedian(data_front_left,msg.data[3]),addValueAndGetMedian(data_front_right,msg.data[2])),addValueAndGetMedian(data_front,msg.data[4]));
	msg_to_send.data = min_data;
	chatter_sonar_front.publish(msg_to_send);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Sonar_filter_node");

	ros::NodeHandle n;
	ros::Subscriber sub_imu = n.subscribe("/pololu/sonars", 1000, callbackPololu);
 	chatter_sonar_front = n.advertise<std_msgs::Float64>("/sonars/front", 1000);
 	chatter_sonar_right = n.advertise<std_msgs::Float64>("/sonars/right", 1000);
 	chatter_sonar_left = n.advertise<std_msgs::Float64>("/sonars/left", 1000);
 
	ros::spin();

  
	return 0;
}
