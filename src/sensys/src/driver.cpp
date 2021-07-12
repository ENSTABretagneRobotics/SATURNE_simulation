#include "serial/serial.h"
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <fstream>
serial::Serial sensys;
std::string port;
int baudrate;
bool firstFrame = false;
bool firstGNSS = false;
double current_lat = 0.;
double current_long = 0.;
int count = 0;
ros::Publisher chatter_data;

std::ofstream file;

std::vector<std::string> split(std::string& str, char c)
{
	std::istringstream stream(str);
	std::string elem;
	std::vector<std::string> elems;
	while (std::getline(stream, elem, c))
	{
		elems.push_back(elem);
	}
	return elems;
}

void callbackFix(const sensor_msgs::NavSatFix& msg)
{
	current_lat = msg.latitude;
	current_long = msg.longitude;
	count = 0;
	firstGNSS = true;
}

void parse(std::string data)
{
	std::replace(data.begin(),data.end(),'E','e');
	std::replace(data.begin(),data.end(),',','.');
	std::vector<std::string> split_data = split(data,';');
	if(split_data.size() == 5)
	{
		if(firstGNSS)
		{
		file << count <<";" << current_lat<<std::setprecision(10)<< ";"<<current_long <<std::setprecision(10)<<";" << std::stod(split_data[4]) <<std::setprecision(10)<< "\n";
		count++;
		std_msgs::Float64MultiArray msg;
		msg.data.resize(4);
		msg.data[0] = count;
		msg.data[1] = current_lat;
		msg.data[2] = current_long;
		msg.data[3] = std::stod(split_data[4]);
		chatter_data.publish(msg);
		}
		else
		{
		    ROS_WARN("Waiting for GNSS");
		}
	}
	else
	{
	    ROS_WARN("Invalid data");
	}

}


bool openSerial(int argc, char **argv)
{
	ROS_INFO("Trying to access %s",port.c_str());
	try
	{
		sensys.open();
		if(sensys.isOpen())
		{
			
			ROS_INFO("Success");
			ROS_INFO("Ready to receive data...");
			std::string buffer = "";

			while (ros::ok())
			{
				if(sensys.available())
				{
				    std::string ch;
					if(sensys.read(ch, 1) == 1)
					{
						if(ch == "\r")
						{	
							if(firstFrame)
							{
							    parse(buffer);
							}
							firstFrame = true;
							buffer = "";
						}
						else
						{
							buffer = buffer + ch;
						}
					}
					else
					{
						ROS_WARN("Failure to read");
					}
				}
				usleep(1);
				ros::spinOnce();
			}
			return true;
		}
		else
		{
			firstFrame = false;
			sensys.close();
			ROS_ERROR("Failure to open. It should have triggered an error, not this.");
			return false;
		}	
	}
	catch (const serial::IOException& e)
	{
		firstFrame = false;
		sensys.close();
		ROS_ERROR("Failure to access %s",port.c_str());
		return false;
	}
	catch (const serial::SerialException& e)
	{
		firstFrame = false;
		sensys.close();
		ROS_ERROR("Unexpected exception catched");
		return false;
	}

	
}




int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Sensys_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
        std::string port =  nh.param<std::string>("port","/dev/ttyS1");
	baudrate =  nh.param<int>("baudrate",9600);
	sensys.setPort(port);
	sensys.setBaudrate(baudrate);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
	sensys.setTimeout(timeout);
	ros::Subscriber sub_fix = n.subscribe("/fix", 1000, callbackFix);
	chatter_data = n.advertise<std_msgs::Float64MultiArray>("/data", 1000);
	
	std::string name = "log_data." + std::to_string(ros::Time::now().toSec())+".csv";
  	file.open(name, std::ios::out | std::ios::app );
  
	while(openSerial(argc,argv) == false)
	{
		usleep(1000000);
	}
	sensys.close();
  	file.close();
	return 0;

}
