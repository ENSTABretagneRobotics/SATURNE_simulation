#include "ros/ros.h"
#include <cmath>
#include "UTM.h"
#include "sensor_msgs/NavSatFix.h"
#include "regul/msg_estim.h"
#include "regul/msg_mission.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"


struct mission
{
	int type; //0 : suivi de waypoint //1 : suivi de ligne //2: cercle

	float p0x;
	float p0y;
	float p1x;
	float p1y;
	float p2x;
	float p2y;

	float offset_validation;
	int sens;

	float r = 0.;
	int eps;
};


std::vector<mission> liste_missions;
mission current_mission;
int current_mission_type;
int current_mission_id = 0;


float dist_validation = 1.5;
bool cond_angul = false;

ros::Publisher chatter_draw;

float estim_x,estim_y;
double estim_theta;
bool estim_correct;

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}

float dist(float x1, float y1, float x2, float y2)
{
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}


mission new_mission_wp(float ax, float ay, float bx, float by, float offset_validation)
{
	mission mi;
	mi.type = 0;
	mi.p0x = ax;
	mi.p0y = ay;
	mi.p1x = bx;
	mi.p1y = by;
	mi.offset_validation = offset_validation;

	return mi;
}

mission new_mission_lf(float ax, float ay, float bx, float by, float offset_validation)
{
	mission mi;
	mi.type = 1;
	mi.p0x = ax;
	mi.p0y = ay;
	mi.p1x = bx;
	mi.p1y = by;
	mi.offset_validation = offset_validation;

	return mi;
}

mission new_mission_circle(float ax, float ay, float bx, float by,float r_offset,int eps,float cx, float cy, float dx, float dy, float sens)
{
	mission mi;
	mi.type = 2;

	mi.p0x = (ax+bx)/2.;
	mi.p0y = (ay+by)/2.;

	mi.p1x = cx;
	mi.p1y = cy;
	mi.p2x = dx;
	mi.p2y = dy;

	mi.sens = sens;
	mi.offset_validation = 0.;

	mi.r = r_offset + dist(ax,ay,bx,by)/2.;
	mi.eps = eps;

	return mi;
}

void sendMissions()
{
	for(int i = 0; i < liste_missions.size(); i++)
	{
		regul::msg_mission msn;
		msn.type = liste_missions[i].type;
		msn.x0 = liste_missions[i].p0x;
		msn.y0 = liste_missions[i].p0y;
		msn.x1 = liste_missions[i].p1x;
		msn.y1 = liste_missions[i].p1y;
		msn.r = liste_missions[i].r;
		chatter_draw.publish(msn);
	}
}

void callbackAsk(const std_msgs::Bool &msg)
{
	sendMissions();
}

float dist_to_b(float ax,float ay,float bx,float by)
{
	float norm = sqrt((bx - ax)*(bx - ax)+(by - ay)*(by - ay));
	return ((bx - ax)*(estim_x - bx)+(by - ay)*(estim_y - by))/norm;
}

void callbackEstim(const regul::msg_estim& msg)
{
	estim_x = msg.x;
	estim_y = msg.y;
	estim_theta = msg.theta;
	estim_correct = msg.correct;
}

std::pair<double,double> phi0(float p1, float p2)
{       
    float v1,v2;
    v1 = -(p1*p1*p1+p2*p2*p1-p1+p2);
    v2 = -(p2*p2*p2+p1*p1*p2-p1-p2);
    return std::make_pair(v1,v2);
}
 
std::pair<double,double> phi(float p1, float p2,float c1,float c2,float r,int eps)
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


bool loadMission()
{
	if(current_mission_id < (liste_missions.size()-1))
	{
		current_mission = liste_missions[current_mission_id];
		current_mission_type = current_mission.type;
		if(current_mission_type == 0)
		{
			ROS_INFO("Debut d'une mission (numero : %i) de suivi de waypoint.",current_mission_id);
		}
		else if(current_mission_type == 1)
		{
			ROS_INFO("Debut d'une mission (numero : %i) de suivi de ligne.",current_mission_id);
		}
		else if(current_mission_type == 2)
		{
			cond_angul = false;
			ROS_INFO("Debut d'une mission (numero : %i) de suivi de cercle.",current_mission_id);
		}
		else
		{
			current_mission_id++;
			loadMission();
		}

		return true;
	}
	else
	{
		ROS_INFO("Fin des missions.");
		return false;
	}
}

bool endMission()
{
	ROS_INFO("Fin de la mission actuelle.");
	current_mission_id++;
	return loadMission();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Obj_node");
	ros::NodeHandle n;

	ros::Subscriber sub_estim = n.subscribe("/estim", 1000, callbackEstim);
 	ros::Publisher chatter_obj = n.advertise<geometry_msgs::Twist>("/cmd_vel_yaw", 1000);
 	chatter_draw = n.advertise<regul::msg_mission>("/draw_wp", 1000);
 	ros::Subscriber sub_ask = n.subscribe("/ask_points", 1000, callbackAsk);

 	ros::Rate loop_rate(10);
/*
 	float lat0 = 48.418224;
 	float lon0 = -4.473607;

 	float lat1 = 48.418192; 
 	float lon1 = -4.474125;

 	float lat3 = 48.418844;
 	float lon3 = -4.473665;

 	float lat2 = 48.418804; 
 	float lon2 = -4.474168;
*/
 	float lat0 = 48.418635; 
 	float lon0 = -4.473634;
 
 	float lat1 = 48.418624; 
 	float lon1 = -4.474211;

 	float lat2 = 48.418857; 
 	float lon2 = -4.474267;
  
 	float lat3 = 48.418900; 
 	float lon3 = -4.473706;

 	float ax,ay,bx,by,cx,cy,dx,dy;
 	int nb_rails = 3;

	LatLonToUTMXY(lat0,lon0,0,ay,ax);
    LatLonToUTMXY(lat1,lon1,0,by,bx);
    LatLonToUTMXY(lat2,lon2,0,cy,cx);
    LatLonToUTMXY(lat3,lon3,0,dy,dx);

    std::vector<std::pair<float,float>> ligne_0;
    std::vector<std::pair<float,float>> ligne_1;

    std::pair<float,float> vect_0 = {(dx-ax)/(2.*(float)nb_rails),(dy-ay)/(2.*(float)nb_rails)};
    std::pair<float,float> vect_1 = {(cx-bx)/(2.*(float)nb_rails),(cy-by)/(2.*(float)nb_rails)};

    for(int i = 0; i < 2*nb_rails ;i++)
    {
	    std::pair<float,float> p0;
	    std::pair<float,float> p1;
	    
	    p0.first = ax + vect_0.first*i;
		p0.second = ay + vect_0.second*i;
	    p1.first = bx + vect_1.first*i;
		p1.second = by + vect_1.second*i;

		ligne_0.push_back(p0);
		ligne_1.push_back(p1);
    }


    for(int i = 0; i < ligne_1.size() - 3; i++)
    {
    	liste_missions.push_back(new_mission_lf(ligne_0[i].first,ligne_0[i].second,ligne_1[i].first,ligne_1[i].second,0.));
    	liste_missions.push_back(new_mission_circle(ligne_1[i].first,ligne_1[i].second,ligne_1[i+3].first,ligne_1[i+3].second,0.,1,ligne_1[i+3].first,ligne_1[i+3].second,ligne_0[i+3].first,ligne_0[i+3].second,1.));
    	liste_missions.push_back(new_mission_lf(ligne_1[i+3].first,ligne_1[i+3].second,ligne_0[i+3].first,ligne_0[i+3].second,0.));
    	liste_missions.push_back(new_mission_circle(ligne_0[i+3].first,ligne_0[i+3].second,ligne_0[i+1].first,ligne_0[i+1].second,0.,1,ligne_0[i+1].first,ligne_0[i+1].second,ligne_1[i+1].first,ligne_1[i+1].second,-1.));
    	
   	}
   	//liste_missions.pop_back();



 	//liste_missions.push_back(new_mission_wp(48.418114, -4.473455));
/*
 	liste_missions.push_back(new_mission_wp(48.418114, -4.473455));
	liste_missions.push_back(new_mission_lf(48.418114, -4.473455, 48.418116, -4.474276));
	liste_missions.push_back(new_mission_wp(48.418196, -4.474287));
	liste_missions.push_back(new_mission_lf(48.418196, -4.474287, 48.418203, -4.473475));
	liste_missions.push_back(new_mission_wp(48.418292, -4.473496));
	liste_missions.push_back(new_mission_lf(48.418292, -4.473496, 48.418276, -4.474299));
	liste_missions.push_back(new_mission_wp(48.418356, -4.474310));
	liste_missions.push_back(new_mission_lf(48.418356, -4.474310, 48.418382, -4.473516));
	liste_missions.push_back(new_mission_wp(48.418471, -4.473536));
	liste_missions.push_back(new_mission_lf(48.418471, -4.473536, 48.418436, -4.474321));
	liste_missions.push_back(new_mission_wp(48.418515, -4.474333));
	liste_missions.push_back(new_mission_lf(48.418515, -4.474333, 48.418560, -4.473557));
	liste_missions.push_back(new_mission_wp(48.418649, -4.473577));
	liste_missions.push_back(new_mission_lf(48.418649, -4.473577, 48.418595, -4.474344));
	liste_missions.push_back(new_mission_wp(48.418675, -4.474355));
	liste_missions.push_back(new_mission_lf(48.418675, -4.474355, 48.418739, -4.473597));
	liste_missions.push_back(new_mission_wp(48.418828, -4.473618));
	liste_missions.push_back(new_mission_lf(48.418828, -4.473618, 48.418755, -4.474367));
	liste_missions.push_back(new_mission_wp(48.418835, -4.474378));
	liste_missions.push_back(new_mission_lf(48.418835, -4.474378, 48.418917, -4.473638));
	liste_missions.push_back(new_mission_wp(48.418114, -4.473455));
	*/

	/*liste_missions.push_back(new_mission_wp(48.418114, -4.473000, 48.418114, -4.473455));
	liste_missions.push_back(new_mission_lf(48.418114, -4.473455, 48.418116, -4.474276));
	//liste_missions.push_back(new_mission_wp(48.418116, -4.474276, 48.418196, -4.474287));
	liste_missions.push_back(new_mission_circle(48.418116, -4.474276, 48.418196, -4.474287,1.5,1,48.418196, -4.474287, 48.418203, -4.47347));
	liste_missions.push_back(new_mission_lf(48.418196, -4.474287, 48.418203, -4.473475));
	liste_missions.push_back(new_mission_wp(48.418203, -4.473475, 48.418292, -4.473496));
	liste_missions.push_back(new_mission_lf(48.418292, -4.473496, 48.418276, -4.474299));
	liste_missions.push_back(new_mission_wp(48.418276, -4.474299, 48.418356, -4.474310));
	liste_missions.push_back(new_mission_lf(48.418356, -4.474310, 48.418382, -4.473516));
	liste_missions.push_back(new_mission_wp(48.418382, -4.473516, 48.418471, -4.473536));
	liste_missions.push_back(new_mission_lf(48.418471, -4.473536, 48.418436, -4.474321));
	liste_missions.push_back(new_mission_wp(48.418436, -4.474321, 48.418515, -4.474333));
	liste_missions.push_back(new_mission_lf(48.418515, -4.474333, 48.418560, -4.473557));
	liste_missions.push_back(new_mission_wp(48.418560, -4.473557, 48.418649, -4.473577));
	liste_missions.push_back(new_mission_lf(48.418649, -4.473577, 48.418595, -4.474344));
	liste_missions.push_back(new_mission_wp(48.418595, -4.474344, 48.418675, -4.474355));
	liste_missions.push_back(new_mission_lf(48.418675, -4.474355, 48.418739, -4.473597));
	liste_missions.push_back(new_mission_wp(48.418739, -4.473597, 48.418828, -4.473618));
	liste_missions.push_back(new_mission_lf(48.418828, -4.473618, 48.418755, -4.474367));
	liste_missions.push_back(new_mission_wp(48.418755, -4.474367, 48.418835, -4.474378));
	liste_missions.push_back(new_mission_lf(48.418835, -4.474378, 48.418917, -4.473638));
	liste_missions.push_back(new_mission_lf(48.418917, -4.473638, 48.418114, -4.473455));

	*/


 	loadMission();

	bool continue_missions = true;
 	geometry_msgs::Twist msg;

 	double speed = 2.;

    while (ros::ok() and continue_missions)
    {
    	msg.linear.x = 0.;
		msg.angular.z = 0.;

    	if(current_mission_type == 1) //Line following
    	{
		    float dist_b = dist_to_b(current_mission.p0x,current_mission.p0y,current_mission.p1x,current_mission.p1y);
		    //ROS_INFO("%f",dist_b);
			if(dist_b > current_mission.offset_validation  and estim_correct)
			{
				ROS_INFO("Valide à %f m",dist_b);
				continue_missions = endMission();
			}
			else
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
			    msg.linear.x = speed;
				msg.angular.z = atan2(wy,wx);

				chatter_obj.publish(msg);
			}
		}
		else if(current_mission_type == 2) //Circle following
		{
			float dist_b = dist_to_b(liste_missions[current_mission_id-1].p1x,liste_missions[current_mission_id-1].p1y,liste_missions[current_mission_id+1].p0x,liste_missions[current_mission_id+1].p0y);
			if(cond_angul == false)
			{
				float angle = atan2(liste_missions[current_mission_id+1].p1y-liste_missions[current_mission_id+1].p0y,liste_missions[current_mission_id+1].p1x-liste_missions[current_mission_id+1].p0x);
				ROS_INFO("%f %f deg %f deg",dist_b,estim_theta*180./M_PI,angle*180./M_PI);
				if(fabs(sawtooth(estim_theta - angle)) < 0.2)
				{
					cond_angul = true;
					ROS_INFO("Condition angulaire validee");
				}
			}
			else
			{
				ROS_INFO("%f",dist_b);
			}

			/*if(current_mission.sens == 1 and estim_correct and estim_theta > 0.9*M_PI/2. and estim_theta  < 1.1*M_PI/2.)
			{
				continue_missions = endMission();	
			}
			if(current_mission.sens == -1 and estim_correct and estim_theta > -1.1*M_PI/2. and estim_theta  < -0.9*M_PI/2.)
			{
				continue_missions = endMission();	
			}*/
			if(estim_correct and dist_b > current_mission.offset_validation and cond_angul)
			{
				continue_missions = endMission();
				cond_angul = false;	
			}
			std::pair<double,double> w = phi(estim_x,estim_y,current_mission.p0x,current_mission.p0y,current_mission.r,current_mission.eps);
			msg.linear.x = speed;
			msg.angular.z = atan2(w.second,w.first);

			chatter_obj.publish(msg);
		}
		else if(current_mission_type == 0) //Waypoint following
		{

			float dist_b = dist_to_b(current_mission.p0x,current_mission.p0y,current_mission.p1x,current_mission.p1y);
		    //ROS_INFO("%f",dist_b);
			if(dist_b > current_mission.offset_validation and estim_correct)
			{
				continue_missions = endMission();
			}
			else
			{
				float norm = sqrt((current_mission.p1x - current_mission.p0x)*(current_mission.p1x - current_mission.p0x)+(current_mission.p1y - current_mission.p0y)*(current_mission.p1y - current_mission.p0y));
				float det_mat_dist = (current_mission.p1x - current_mission.p0x)*(estim_y - current_mission.p0y) - (current_mission.p1y - current_mission.p0y) * (estim_x - current_mission.p0x);
				float dist_ligne = det_mat_dist/norm;
    			float wx = -2.*(estim_x - current_mission.p1x);
    			float wy = -2.*(estim_y - current_mission.p1y);
    			//ROS_INFO("%f",atan2(wy,wx)*180./M_PI);
			    msg.linear.x = speed;
				msg.angular.z = atan2(wy,wx);

				chatter_obj.publish(msg);
			}
		}
        loop_rate.sleep();
	    ros::spinOnce();
    }
	return 0;
}
