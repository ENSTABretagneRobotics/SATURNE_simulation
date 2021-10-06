#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "roboteq/msg_cmd_esc.h"

/*
Ce code prends en entrée la commande esc du code de Robin et en fait des topics lisible par la simulation gazebo
Also compatible with geometry_msgs::Twist...
*/
ros::Publisher chatter_cmd_left_back, chatter_cmd_right_back, chatter_cmd_left_front, chatter_cmd_right_front;

void callbackCmd_vel(const geometry_msgs::Twist& msg)
{
	double coef = 5;
	double msg_left = coef*std::min(std::max(msg.linear.x-msg.angular.z,-6.0),6.0);
	double msg_right = coef*std::min(std::max(msg.linear.x+msg.angular.z,-6.0),6.0);

	std_msgs::Float64 msg1, msg2, msg3, msg4;
	msg1.data = msg_left;
	msg2.data = msg_right;
	msg3.data = msg_left;
	msg4.data = msg_right;

	chatter_cmd_left_back.publish(msg2);
	chatter_cmd_right_back.publish(msg1);
	chatter_cmd_left_front.publish(msg4);
	chatter_cmd_right_front.publish(msg3);
}

void callbackEstim(const roboteq::msg_cmd_esc& msg)
{
	float cmd_left = msg.cmd_left;
	float cmd_right = msg.cmd_right;




	/*
	le max en consigne était de 200, on voudrait que ça corresponde à 3 m/s
	*/
	float alpha = 3/200.;
	//alpha/=2;

	std_msgs::Float64 msg1, msg2, msg3, msg4;
	msg1.data = alpha*cmd_left;
	msg2.data = alpha*cmd_right;
	msg3.data = alpha*cmd_left;
	msg4.data = alpha*cmd_right;

	//ROS_WARN("Receiving %f and sending %f", cmd_left, msg1.data);

	/**
	chatter_cmd_left_back.publish(msg1);
	chatter_cmd_right_back.publish(msg2);
	chatter_cmd_left_front.publish(msg3);
	chatter_cmd_right_front.publish(msg4);
	**/

	
	chatter_cmd_left_back.publish(msg2);
	chatter_cmd_right_back.publish(msg1);
	chatter_cmd_left_front.publish(msg4);
	chatter_cmd_right_front.publish(msg3);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Roboteq_node");

	ros::NodeHandle n;
	ros::Subscriber sub_estim = n.subscribe("/front/cmd_esc", 1000, callbackEstim);
	ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel", 1000, callbackCmd_vel);

  	chatter_cmd_left_back = n.advertise<std_msgs::Float64>("/mybot/left_back_Wheel_effort_controller/command", 1000);
  	chatter_cmd_right_back = n.advertise<std_msgs::Float64>("/mybot/right_back_Wheel_effort_controller/command", 1000);
  	chatter_cmd_left_front = n.advertise<std_msgs::Float64>("/mybot/left_front_Wheel_effort_controller/command", 1000);
  	chatter_cmd_right_front = n.advertise<std_msgs::Float64>("/mybot/right_front_Wheel_effort_controller/command", 1000);
	ros::spin();
    return 0;

}