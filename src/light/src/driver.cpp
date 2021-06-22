#include "serial/serial.h"
#include "ros/ros.h"
#include <time.h>
#include "std_msgs/Bool.h"

serial::Serial maestro;

short target_lights = 0;
short target_warning = 0;


bool emergency_mode = false;
bool lights_on = false;
bool blinking_on = false;
bool warning_on = false;

short channel_light_0 = 0x0C;
short channel_light_1 = 0x0D;
short channel_warning = 0x0E;

int button_state_0 = 0;
int button_state_1 = 0;
int button_state_2 = 0;


void callbackEmergency(const std_msgs::Bool::ConstPtr& msg)
{
	emergency_mode = msg->data;
}

void callbackBlink(const std_msgs::Bool::ConstPtr& msg)
{
	blinking_on = msg->data;
}

void callbackLights(const std_msgs::Bool::ConstPtr& msg)
{
	lights_on = msg->data;
}

void callbackWarning(const std_msgs::Bool::ConstPtr& msg)
{
	warning_on = msg->data;
}
/*
void cmdCallback(const light::msg_cmd_lights& msg)
{
	target_light_left = (short)((float)(std::min(100,std::max(0,msg.left_power))*(1900.-1100.)/100. +1100.)*4.);
	target_light_right = (short)((float)(std::min(100,std::max(0,msg.right_power))*(1900.-1100.)/100. +1100.)*4.);
	flash_left = msg.left_flash;
	flash_right = msg.right_flash;
}
*/

int readData(int &button_state)
{
	if(maestro.available())
	{
	    uint8_t ch[2];
		if(maestro.read(ch, 2) != 2)
		{
			return 0;
		}
		else
		{
			if((int)(ch[0] + 256*ch[1]) >= 1000)
			{
				button_state = 1;
				return 0;
			}
			else if(button_state)
			{
				button_state = 0;
				return 1;
			}
			else
			{
				return 0;
			}
		}
	}
	return 0;
}


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Light_node");
	ros::NodeHandle nh("~");
    std::string port =  nh.param<std::string>("port","/dev/POLOLU");
	maestro.setPort(port);
	maestro.setBaudrate(9600);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
	maestro.setTimeout(timeout);
	maestro.open();
	std::vector< uint8_t > command_lights_0;
	std::vector< uint8_t > command_lights_1;
	std::vector< uint8_t > command_warning;
	std::vector< uint8_t > get_button_0 = {0x90,0x0F};
	std::vector< uint8_t > get_button_1 = {0x90,0x10};
	std::vector< uint8_t > get_button_2 = {0x90,0x11};
	if(maestro.isOpen())
	{
		ros::NodeHandle n;
		ros::Subscriber sub_lights = n.subscribe("/pololu/lights_on", 1000, callbackLights);
		ros::Subscriber sub_blk = n.subscribe("/pololu/blinking_on", 1000, callbackBlink);
		ros::Subscriber sub_warn = n.subscribe("/pololu/warning_on", 1000, callbackWarning);
		ros::Publisher chatter_b0 = n.advertise<std_msgs::Bool>("/pololu/button_0", 1000);
		ros::Publisher chatter_b1 = n.advertise<std_msgs::Bool>("/pololu/button_1", 1000);
		ros::Publisher chatter_b2 = n.advertise<std_msgs::Bool>("/pololu/button_2", 1000);
		std_msgs::Bool msg;
    	msg.data = true;
		ros::Subscriber sub_emergency = n.subscribe("/emergency", 1000, callbackEmergency);
		ros::Rate loop_rate(15);
		int flash_count = 0;
		while (ros::ok())
		{
			target_lights = 0;
			target_warning = 0;
			if(emergency_mode)
			{
				target_lights = 5040;
				target_warning = 6500;
			}
			else
			{
				flash_count += 10;

				if(flash_count > 100)
				{
					flash_count = 0;
				}
				if(lights_on)
				{
					target_lights = 5040;
				}
				if(blinking_on)
				{
					target_lights =  2000;
					if(flash_count > 1)
					{
						target_lights = 5040;
					}
				}
				if(warning_on)
				{
					target_warning = 6500;
				}
			}
			command_lights_0 = {0x84,channel_light_0,target_lights & 0x7F,(target_lights >> 7) & 0x7F};
			command_lights_1 = {0x84,channel_light_1,target_lights & 0x7F,(target_lights >> 7) & 0x7F};
			command_warning = {0x84,channel_warning,target_warning & 0x7F,(target_warning >> 7) & 0x7F};
			
			maestro.write(command_lights_0);
			maestro.write(command_lights_1);
			maestro.write(command_warning);
			maestro.write(get_button_0);
			if(readData(button_state_0))
			{
				warning_on = !warning_on;
    			chatter_b0.publish(msg);
			}
			maestro.write(get_button_1);
			if(readData(button_state_1))
			{
    			chatter_b1.publish(msg);
			}
			maestro.write(get_button_2);
			if(readData(button_state_2))
			{
    			chatter_b2.publish(msg);
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
		target_lights = 0;
		target_warning = 0;
		command_lights_0 = {0x84,channel_light_0,target_lights & 0x7F,(target_lights >> 7) & 0x7F};
		command_lights_1 = {0x84,channel_light_1,target_lights & 0x7F,(target_lights >> 7) & 0x7F};
		command_warning = {0x84,channel_warning,target_warning & 0x7F,(target_warning >> 7) & 0x7F};
		maestro.write(command_lights_0);
		maestro.write(command_lights_1);
		maestro.write(command_warning);
		maestro.close();
	}
	else
	{
		std::cout << "Echec de l'ouverture" << std::endl;
	}

}