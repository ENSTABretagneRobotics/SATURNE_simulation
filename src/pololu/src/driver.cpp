#include "serial/serial.h"
#include "ros/ros.h"
#include <time.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"


serial::Serial maestro;

short target_warning = 0;
short target_red = 0;
short target_yellow = 0;
short target_green = 0;
short target_alarm = 0;

short channel_warning = 0x08;
short channel_trigger_sonars = 0x09;
short channel_red = 0x0C;
short channel_yellow = 0x0D;
short channel_green = 0x0E;
short channel_alarm = 0x0F;
ros::Time last_msg;
ros::Time last_msg_cmd_vel;
ros::Time last_msg_alarm;
bool mode_alarm = false;


std::vector< uint8_t > get_sonar_0 = {0x90,0x00};
std::vector< uint8_t > get_sonar_1 = {0x90,0x01};
std::vector< uint8_t > get_sonar_2 = {0x90,0x02};
std::vector< uint8_t > get_sonar_3 = {0x90,0x03};
std::vector< uint8_t > get_sonar_4 = {0x90,0x04};

const double facteur_distance = 0.2575;

void callbackWarning(const std_msgs::Int32& msg)
{
	last_msg = ros::Time::now();
}

double readData()
{
	if(maestro.available())
	{
	    uint8_t ch[2];
		if(maestro.read(ch, 2) != 2)
		{
			return -1.;
		}
		else
		{
			return (double)(ch[0] + 256*ch[1])*0.25/facteur_distance;
		}
	}
}

void callbackCmdVel(const geometry_msgs::Twist& msg)
{
	last_msg_cmd_vel = ros::Time::now();
}

void callbackAlarm(const std_msgs::Bool& msg)
{
	mode_alarm = msg.data;
	last_msg_alarm = ros::Time::now();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Light_node");
	ros::NodeHandle nh("~");
	ros::NodeHandle n;
    std::string port =  nh.param<std::string>("port","/dev/ttyS2");
    ros::Publisher chatter_sonars = n.advertise<std_msgs::Float64MultiArray>("/pololu/sonars", 1000);
    std_msgs::Float64MultiArray msg;
    msg.data.resize(5);
	maestro.setPort(port);
	maestro.setBaudrate(57600);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
	maestro.setTimeout(timeout);
	maestro.open();
	std::vector< uint8_t > command_warning;
	std::vector< uint8_t > command_red;
	std::vector< uint8_t > command_yellow;
	std::vector< uint8_t > command_green;
	std::vector< uint8_t > command_alarm;
	if(maestro.isOpen())
	{
		ros::Subscriber sub_warn = n.subscribe("/left_drive/status/motor_temperature", 1000, callbackWarning);
		ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel", 1000, callbackCmdVel);
		ros::Subscriber sub_alarm = n.subscribe("/alarm", 1000, callbackAlarm);
		std::vector< uint8_t >  command_trigger_sonars_0 = {0x84,channel_trigger_sonars,6500 & 0x7F,(6500 >> 7) & 0x7F};
		std::vector< uint8_t >  command_trigger_sonars_1 = {0x84,channel_trigger_sonars,0 & 0x7F,(0 >> 7) & 0x7F};
		ros::Rate loop_rate(10);
		while (ros::ok())
		{

			target_green = 6500;
			if((ros::Time::now()-last_msg).toSec() < 1.5)
		    {
				target_warning = 6500;
			}
			else
			{
				target_warning = 0;
			}
			if((ros::Time::now()-last_msg_cmd_vel).toSec() < 0.5)
		    {
				target_red = 6500;				
				target_yellow = 0;
			}
			else
			{
				target_red = 0;
				target_yellow = 6500;	
			}
			if(mode_alarm and (ros::Time::now()-last_msg_alarm).toSec() < 0.5)
			{
				target_alarm = 6500;
			}
			else
			{
				target_alarm = 0;
			}
			
			command_warning = {0x84,channel_warning,target_warning & 0x7F,(target_warning >> 7) & 0x7F};

			command_red = {0x84,channel_red,target_red & 0x7F,(target_red >> 7) & 0x7F};
			command_yellow = {0x84,channel_yellow,target_yellow & 0x7F,(target_yellow >> 7) & 0x7F};
			command_green = {0x84,channel_green,target_green & 0x7F,(target_green >> 7) & 0x7F};
			command_alarm = {0x84,channel_alarm,target_alarm & 0x7F,(target_alarm >> 7) & 0x7F};

			maestro.write(command_warning);

			maestro.write(command_red);
			maestro.write(command_yellow);
			maestro.write(command_green);
			maestro.write(command_alarm);
			/*maestro.write(command_trigger_sonars_0);
			usleep(25);
			maestro.write(command_trigger_sonars_1);
			*/maestro.write(get_sonar_0);
			msg.data[0] = readData();
			maestro.write(get_sonar_1);
			msg.data[1] = readData();
			maestro.write(get_sonar_2);
			msg.data[2] = readData();
			maestro.write(get_sonar_3);
			msg.data[3] = readData();
			maestro.write(get_sonar_4);
			msg.data[4] = readData();

			chatter_sonars.publish(msg);

			ros::spinOnce();
			loop_rate.sleep();
		}
		target_warning = 0;
		target_red = 0;
		target_yellow = 0;
		target_green = 0;
		target_alarm = 0;
		command_warning = {0x84,channel_warning,target_warning & 0x7F,(target_warning >> 7) & 0x7F};
		command_red = {0x84,channel_red,target_red & 0x7F,(target_red >> 7) & 0x7F};
		command_yellow = {0x84,channel_yellow,target_yellow & 0x7F,(target_yellow >> 7) & 0x7F};
		command_green = {0x84,channel_green,target_green & 0x7F,(target_green >> 7) & 0x7F};
		command_alarm = {0x84,channel_alarm,target_alarm & 0x7F,(target_alarm >> 7) & 0x7F};
		maestro.write(command_red);
		maestro.write(command_yellow);
		maestro.write(command_green);
		maestro.write(command_alarm);
		maestro.write(command_warning);
		maestro.close();
	}
	else
	{
		std::cout << "Echec de l'ouverture" << std::endl;
	}

}
