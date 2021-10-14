#include "serial/serial.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"


#define MAX_UBX_STRUCT_LENGTH 120
serial::Serial uart_gnss;
std::string port;
int baudrate;
int UBX = 0;	
std::string typeOfIncomingUBXMessage = "";
int lengthOfIncomingUBXMessage = 0;
ros::Publisher chatter_fix ;

struct UBX_RELPOSNED
{
	uint8_t version;
	uint8_t reserved0;
	uint16_t refStationId;
	uint32_t iTOW;
	int32_t relPosN;
	int32_t relPosE;
	int32_t relPosD;
	int32_t relPosLenght;
	int32_t relPosHeading;
	uint32_t reserved1; //uint8_t reserved1[4];
	int8_t relPosHPN;
	int8_t relPosHPE;
	int8_t relPosHPD;
	int8_t relPosHPLength;
	uint32_t accN;
	uint32_t accE;
	uint32_t accD;
	uint32_t accLength;
	uint32_t accHeading;
	uint32_t reserved2;//uint8_t reserved2[4];
	char flags[4];
};

uint8_t parseU8(std::string data,int pos)
{
	return *(uint8_t*)data.substr(pos,sizeof(uint8_t)).c_str();
}
uint16_t parseU16(std::string data,int pos)
{
	return *(uint16_t*)data.substr(pos,sizeof(uint16_t)).c_str();
}
uint32_t parseU32(std::string data,int pos)
{
	return *(uint32_t*)data.substr(pos,sizeof(uint32_t)).c_str();
}
uint64_t parseU64(std::string data,int pos)
{
	return *(uint64_t*)data.substr(pos,sizeof(uint64_t)).c_str();
}
int8_t parseI8(std::string data,int pos)
{
	return *(int8_t*)data.substr(pos,sizeof(int8_t)).c_str();
}
int16_t parseI16(std::string data,int pos)
{
	return *(int16_t*)data.substr(pos,sizeof(int16_t)).c_str();
}
int32_t parseI32(std::string data,int pos)
{
	return *(int32_t*)data.substr(pos,sizeof(int32_t)).c_str();
}
int64_t parseI64(std::string data,int pos)
{
	return *(int64_t*)data.substr(pos,sizeof(int64_t)).c_str();
}

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

int checksumNMEA(std::string data)
{
	int checksum = 0;
	for(int i = 0; i < data.length(); i++)
	{
		if(data.at(i)!= '$')
		{
			if(data.at(i) == '*')
			{
				return checksum;
			}
			else
			{
				checksum ^= (char)data.at(i);
			}
		}
	}
	return checksum;
}

void parseNMEA(std::string data)
{
	//ROS_INFO("%s",data.c_str());
	int checksum = checksumNMEA(data);
	std::vector<std::string> split_nmea_checksum = split(data,'*');
	if(split_nmea_checksum.size() >= 2)
	{
		split_nmea_checksum[1] = split_nmea_checksum[1].substr(0,2);
		std::vector<std::string> split_nmea = split(split_nmea_checksum[0],',');
		if(split_nmea.size() >= 2)
		{	
			if(std::stoi(split_nmea_checksum[1],0,16) == checksum) //TODO :Add a try for stoi
			{
				if(split_nmea[0].substr(3,3) == "GGA") //TODO Check size = 14 et stod secu + covariance
				{
					sensor_msgs::NavSatFix msg;
					msg.header.stamp = ros::Time::now();

					if(split_nmea[2].length() > 2)
					{
						msg.latitude = std::stod(split_nmea[2].substr(0,2)) + std::stod(split_nmea[2].substr(2))/60.;
						if(split_nmea[3] == "S")
						{
							msg.latitude = -msg.latitude;
						}
					}
					else
					{
						msg.latitude = std::nan("1");
					}

					if(split_nmea[4].length() > 3)
					{
						msg.longitude = -(std::stod(split_nmea[4].substr(0,3)) + std::stod(split_nmea[4].substr(3))/60.);//signe ?
						if(split_nmea[3] == "W")
						{
							msg.longitude = -msg.longitude;
						}
					}
					else
					{
						msg.longitude = std::nan("1");
					}
					if(split_nmea[9].length() > 1 && split_nmea[11].length() > 1)
					{
						msg.altitude = std::stod(split_nmea[9]) + std::stod(split_nmea[11]);
					}
					else
					{
						msg.altitude = std::nan("1");
					}
					msg.status.status = stoi(split_nmea[6]);
					chatter_fix.publish(msg);
					/*for(int i = 0;i < split_nmea.size() - 1;i++)
					{

						std::cout << split_nmea[i] << " ";
					}
					std::cout << std::endl;
					*/
				}
				/*


				*/
			}
			else
			{
				ROS_WARN("Invalid NMEA checksum");
			}
		}
		else
		{
			ROS_WARN("Invalid data: not enought data");
		}
	}
	else
	{
		std::cout << data << std::endl;
		ROS_WARN("Invalid data: no checksum");
	}
	
}

void parseUBX(std::string intro,std::string data)
{
	if(typeOfIncomingUBXMessage == "RELPOSNED")
	{
		const char *buffer_intro = intro.c_str();
		const char *buffer = data.c_str();
		UBX_RELPOSNED msg;
		msg.version = parseU8(data,0);
		msg.reserved0 = parseU8(data,1);
		msg.refStationId = parseU16(data,2);
		msg.iTOW = parseU32(data,4); //DO NOT USE ME FOR ANYTHING OTHER THAN DEBUGGING (see ZED-F9P Integration Manual 3.10.3 iTOW timestamps)
		msg.relPosN = parseI32(data,8);
		msg.relPosE = parseI32(data,12);
		msg.relPosD = parseI32(data,16);
		msg.relPosLenght = parseI32(data,20);
		msg.relPosHeading = parseI32(data,24);
		msg.reserved1 = parseU32(data,28);
		msg.relPosHPN = parseI8(data,32);
		msg.relPosHPE = parseI8(data,33);
		msg.relPosHPD = parseI8(data,34);
		msg.relPosHPLength = parseI8(data,35);
		msg.accN = parseU32(data,36);
		msg.accE = parseU32(data,40);
		msg.accD = parseU32(data,44);
		msg.accLength = parseU32(data,48);
		msg.accHeading = parseU32(data,52);
		msg.reserved2 = parseU32(data,56);
		strcpy(msg.flags,data.substr(60,4).c_str());
		uint8_t ck_a = parseU8(data,64);
		uint8_t ck_b = parseU8(data,65);

		uint8_t CK_A_calc = 0;
		uint8_t CK_B_calc = 0;
		//Checksum calculation
		for (int i = 0; i < intro.length(); i++)
		{
			CK_A_calc = CK_A_calc + buffer_intro[i];
			CK_B_calc = CK_B_calc + CK_A_calc;
		}
		for (int i = 0; i < lengthOfIncomingUBXMessage - 2; i++)
		{
			CK_A_calc = CK_A_calc + buffer[i];
			CK_B_calc = CK_B_calc + CK_A_calc;
		}
		if(!(CK_A_calc == ck_a && CK_B_calc == ck_b))
		{
			ROS_WARN("Invalid UBX checksum");
		}
		else
		{
			std::cout<< msg.iTOW << std::endl;
			std::cout<< msg.relPosHeading << std::endl;
			std::cout<< msg.relPosLenght + msg.relPosHPLength << std::endl;	
		}



	}
}

std::pair<int,std::string> getIncomingUBXMessageLenghtAndType(std::string data)
{
	int length = (int)parseU16(data,2);
	if(data[0] == (char)0x01 && data[1] == (char)0x3c) //RELPOSNED
	{
		return std::make_pair(length,"RELPOSNED");
	}
	else
	{
		ROS_WARN("UNKNOWN UBX MESSAGE");
		return std::make_pair(length,"");
	}
}

bool openSerial(int argc, char **argv)
{
	ROS_INFO("Trying to access %s",port.c_str());
	try
	{
		uart_gnss.open();
		if(uart_gnss.isOpen())
		{
			
			ROS_INFO("Success");
			ROS_INFO("Ready to receive data...");
			std::string buffer = "";
			std::string msg_intro = "";
					
			while (ros::ok())
			{
				if(uart_gnss.available())
				{
				    std::string ch;
					if(uart_gnss.read(ch, 1) == 1)
					{
						if(ch == "$" && UBX == 0 )
						{	
							if(buffer.length() != 0)
							{
								parseNMEA(buffer);
							}
							buffer = ch;
						}
						else if(ch[0] == (char)0xb5 && UBX == 0)
						{
							std::string ch2;
							if(uart_gnss.read(ch2, 1) == 1)
							{
								if(ch2[0] == (char)0x62)
								{	
									if(buffer.length() != 0)
									{
										parseNMEA(buffer);
									}
									buffer = "";
									UBX = 1;
								}
								else
								{
									buffer = buffer + ch2;
								}
							}
							else
							{
								buffer = buffer + ch;
							}	
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
					if(UBX == 1 && buffer.length() == 4)
					{
						std::pair<int,std::string> l_t = getIncomingUBXMessageLenghtAndType(buffer);
						msg_intro = buffer;
						buffer = "";
						if(l_t.second == "" || l_t.first <= 0 || l_t.first > MAX_UBX_STRUCT_LENGTH)
						{
							UBX = 0;
							lengthOfIncomingUBXMessage = 0;
							typeOfIncomingUBXMessage = "";
						}
						else
						{
							lengthOfIncomingUBXMessage = l_t.first + 2; //+2 for checksum
							typeOfIncomingUBXMessage = l_t.second;
							UBX = 2;
						}

					}
					else if(UBX == 2 && buffer.length() == lengthOfIncomingUBXMessage)
					{
						parseUBX(msg_intro,buffer);
						msg_intro = "";
						buffer = "";
						UBX = 0;
						lengthOfIncomingUBXMessage = 0;
						typeOfIncomingUBXMessage = "";
					}
				}
				//ros::Duration(0.000001).sleep();
			}
			return true;
		}
		else
		{
			uart_gnss.close();
			ROS_ERROR("Failure to open. It should have triggered an error, not this.");
			return false;
		}	
	}
	catch (const serial::IOException& e)
	{
		uart_gnss.close();
		ROS_ERROR("Failure to access %s",port.c_str());
		return false;
	}
	catch (const serial::SerialException& e)
	{
		uart_gnss.close();
		ROS_ERROR("Unexpected exception catched");
		return false;
	}

	
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Simple_GNSS_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
    port =  nh.param<std::string>("port","/dev/ttyS1");
	baudrate =  nh.param<int>("baudrate",9600);
	uart_gnss.setPort(port);
	uart_gnss.setBaudrate(baudrate);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
	uart_gnss.setTimeout(timeout);
	chatter_fix = n.advertise<sensor_msgs::NavSatFix>("/fix", 1000);
	while(openSerial(argc,argv) == false)
	{
		ros::Duration(1).sleep();
	}
	uart_gnss.close();

	return 0;

}
