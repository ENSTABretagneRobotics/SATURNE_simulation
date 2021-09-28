#include "ros/ros.h"
#include <cmath>
#include "UTM.h"
#include "sensor_msgs/NavSatFix.h"
#include "regul/msg_estim.h"
#include "regul/msg_obj.h"
#include "regul/msg_mission.h"
#include "std_msgs/Bool.h"

struct mission
{
	int type; //0 : suivi de waypoint //1 : suivi de ligne //2: cercle //3: arc

	float p0x = 0;
	float p0y = 0;
	float p1x = 0; //=cap_sortie pour envoyer l'info facilement
	float p1y = 0;

	//var suivi de waypoint
	float offset_validation = 0;
	int sens = 0; // 1: si sens direct  | -1: si sens indirect (sens est parfois appelé eps)
	float cap_sortie = 0;

	float r = 0.; //rayon du cercle à suivre
};


mission current_mission;
int current_mission_type;
//int current_mission_id = 0;
float angl_sortie;

float Ox, Oy; // orgine du répère
float estim_x,estim_y;
double estim_theta;

std::pair<double,double> phi0(float p1, float p2)
{       
    float v1,v2;
    v1 = -(p1*p1*p1+p2*p2*p1-p1+p2);
    v2 = -(p2*p2*p2+p1*p1*p2-p1-p2);
    return std::make_pair(v1,v2);
}
 
std::pair<double,double> phi(float p1, float p2, float c1, float c2, float r, float eps)
{
    float x1 = p1-c1;
    float x2 = p2-c2;
    float y1 = (1./r)*x1;
    float y2 = (1./(r*(float)eps))*x2;
    std::pair<double,double> z = phi0(y1,y2);
    float v1 = (1./r)*z.first;
    float v2 = (1./(r*(float)eps))*z.second;
    return std::make_pair(v1,v2);
}

void callbackEstim(const regul::msg_estim& msg)
{
	estim_x = msg.x;
	estim_y = msg.y;
	estim_theta = msg.theta;
}

void callbackCurrent_mission(const regul::msg_mission& msg)
{
	//ROS_INFO("Objective Node : Receive new objective");

	if (msg.type==0) //waypoint
	{
		current_mission.type=msg.type;
		current_mission.p0x=msg.x0;
		current_mission.p0y=msg.y0;
	}

	if (msg.type==1) //Line following
	{
		current_mission.type=msg.type;
		current_mission.p0x=msg.x0;
		current_mission.p0y=msg.y0;
		current_mission.p1x=msg.x1;
		current_mission.p1y=msg.y1;
	}
	
	if (msg.type==2 || msg.type==3) //Circle ou arc, même contrôleur
	{
		current_mission.type = 2;

		current_mission.p0x = msg.x0;
		current_mission.p0y = msg.y0;

		current_mission.sens = msg.y1;
		current_mission.r = msg.r;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Obj_node");
	ros::NodeHandle n;
	ROS_INFO("Objective Node : Start Objectif node");

	ros::Subscriber sub_estim = n.subscribe("/estim", 1000, callbackEstim);
	ros::Subscriber sub_current_mission = n.subscribe("/currentMission", 1000, callbackCurrent_mission);
 	ros::Publisher chatter_obj = n.advertise<regul::msg_obj>("/obj", 1000);

 	ros::Rate loop_rate(10);

	bool continue_missions = true;
 	regul::msg_obj msg;

 	while (ros::ok() && continue_missions)
    {
		current_mission_type = current_mission.type;
    	if(current_mission_type == 1) //Line following
    	{
			//float angle_ligne = atan2(current_mission.p2y-current_mission.p1y,current_mission.p2y-current_mission.p1x);
			float norm = sqrt((current_mission.p1x - current_mission.p0x)*(current_mission.p1x - current_mission.p0x)+(current_mission.p1y - current_mission.p0y)*(current_mission.p1y - current_mission.p0y));
			float det_mat_dist = (current_mission.p1x - current_mission.p0x)*(estim_y - current_mission.p0y) - (current_mission.p1y - current_mission.p0y) * (estim_x - current_mission.p0x);
			float dist_ligne = det_mat_dist/norm;
			//float heading_calc =  angle_ligne-atan(dist_ligne/4.);   	
		    //msg.heading = heading_calc;
		    //msg.mode_heading = true;
			float nx,ny,wx,wy;
			nx = -(current_mission.p1y - current_mission.p0y);
			ny = (current_mission.p1x - current_mission.p0x);
			float norm1 = sqrt(nx*nx+ny*ny);
			nx = nx / norm1;
			ny = ny / norm1;
			float coeff = 0.1;
			if(dist_ligne < 1.5)
			{
				coeff = (1.5-dist_ligne)/2.+0.1;
			}
			if(coeff > 0.5)
			{
				coeff = 0.5;
			}
			wx = -2.*((nx*nx)*(estim_x - current_mission.p0x)+(nx*ny)*(estim_y - current_mission.p0y)) - coeff*(estim_x - current_mission.p1x);
			wy = -2.*((nx*ny)*(estim_x - current_mission.p0x) + (ny*ny)*(estim_y - current_mission.p0y)) - coeff*(estim_y - current_mission.p1y);
			//ROS_INFO("%f",atan2(wy,wx)*180./M_PI);
			msg.heading = atan2(wy,wx);
		    msg.mode_heading = true;

			chatter_obj.publish(msg);
			
		}
		else if(current_mission_type == 2) //Circle following
		{
			
			std::pair<double,double> w = phi(estim_x, estim_y, current_mission.p0x, current_mission.p0y, current_mission.r, current_mission.sens);
			msg.heading = atan2(w.second,w.first);
		    msg.mode_heading = true;

			chatter_obj.publish(msg);
		}
		else if(current_mission_type == 0) //Waypoint following
		{

			float norm = sqrt((current_mission.p1x - current_mission.p0x)*(current_mission.p1x - current_mission.p0x)+(current_mission.p1y - current_mission.p0y)*(current_mission.p1y - current_mission.p0y));
			float det_mat_dist = (current_mission.p1x - current_mission.p0x)*(estim_y - current_mission.p0y) - (current_mission.p1y - current_mission.p0y) * (estim_x - current_mission.p0x);
			float dist_ligne = det_mat_dist/norm;
			float wx = -2.*(estim_x - current_mission.p1x);
			float wy = -2.*(estim_y - current_mission.p1y);
			//ROS_INFO("%f",atan2(wy,wx)*180./M_PI);
			msg.heading = atan2(wy,wx);
		    msg.mode_heading = true;

			chatter_obj.publish(msg);
			
		}else{
			ROS_INFO("Objective Node : current_mission_id non reconnu");
		}
        loop_rate.sleep();
	    ros::spinOnce();
    }

	return 0;
}
