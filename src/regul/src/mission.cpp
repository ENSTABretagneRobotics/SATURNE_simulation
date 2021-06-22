#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

#include "UTM.h"

#include "sensor_msgs/NavSatFix.h"
#include "regul/msg_estim.h"
#include "regul/msg_mission.h"
#include "std_msgs/Bool.h"

using namespace std;

struct mission{
	int type; //0 : suivi de waypoint //1 : suivi de ligne //2: cercle

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

ros::Publisher chatter_draw;
ros::Publisher chatter_mission;

float dist_validation = 1.5;
bool cond_angul = false;
int r_offset = 0; //de combien on augmente la taille des cercles pour avoir quelque chose de propre

std::vector<mission> l_objectifs;
mission current_mission;
int current_mission_type;
int current_mission_id = 0;

float estim_x,estim_y;
double estim_theta;
bool estim_correct;

void callbackEstim(const regul::msg_estim& msg){
	estim_x = msg.x;
	estim_y = msg.y;
	estim_theta = msg.theta;
	estim_correct = msg.correct;
}

float dist_to_b(float ax,float ay,float bx,float by){
	float norm = sqrt((bx - ax)*(bx - ax)+(by - ay)*(by - ay));
	return ((bx - ax)*(estim_x - bx)+(by - ay)*(estim_y - by))/norm;
}

float dist(float x1, float y1, float x2, float y2){
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

double sawtooth(double x){
	return 2.*atan(tan(x/2.));
}

mission new_mission_wp(float ax, float ay, float bx, float by, float offset_validation){
	mission mi;
	mi.type = 0;
	mi.p0x = ax;
	mi.p0y = ay;
	mi.p1x = bx;
	mi.p1y = by;
	mi.offset_validation = offset_validation;

	return mi;
}

mission new_mission_lf(float ax, float ay, float bx, float by, float offset_validation){
	mission mi;
	mi.type = 1;
	mi.p0x = ax;
	mi.p0y = ay;
	mi.p1x = bx;
	mi.p1y = by;
	mi.offset_validation = offset_validation;

	return mi;
}

mission new_mission_circle(float ax, float ay, float bx, float by, float r_offset, int sens){
	mission mi;
	mi.type = 2;

	mi.p0x = (ax+bx)/2.;
	mi.p0y = (ay+by)/2.;

	mi.p1x = 0;
	mi.p1y = sens;

	mi.r = r_offset + dist(ax,ay,bx,by)/2.;

	return mi;
}

int sign(float a){
	if(a<0){
		return -1;
	}
	return 1;
}

mission new_mission_arc(float cx, float cy, float cap_entre, float cap_sortie, float r){
	mission mi;
	mi.type = 3;

	mi.p0x = cx;
	mi.p0y = cy;

	int sens;
	sens = sign(sin(cap_sortie-cap_entre));

	mi.p1x = cap_sortie + sens*M_PI/4.;
	mi.p1y = sens;

	mi.r = r_offset + r;

	return mi;
}

void read(const string& filename){
	/*
	float lat0 = 48.418476; 
 	float lon0 =  -4.473952;
 	float ax_ref,ay_ref;
	LatLonToUTMXY(lat0,lon0,0,ay_ref,ax_ref);
	ax_ref = 0.;
	ay_ref = 0.;

	
	ifstream f(filename);
	if (!f.is_open()) cout<<"Erreur à l'ouverture du fichier : "<<filename<<endl;
	else
	{
		string m_type;
		size_t pos = -1;

		float ax, ay, bx, by;
		float r;
		string line;
		while (getline(f, line))
		{
			//printf("%s\n", line.c_str());
			
			istringstream row(line);
			row >> m_type;

			row >> ay >> ax >> by >> bx >> r;
			
			if(m_type == "coords"){
				lat0 = ay; 
 				lon0 = ax;
				LatLonToUTMXY(lat0,lon0,0,ay_ref,ax_ref);
			}else if(m_type == "waypoint"){
				ax += ax_ref;
				ay += ay_ref;
				bx += ax_ref;
				by += ay_ref;
		    	l_objectifs.push_back(new_mission_wp(ax, ay, bx, by, 0.));
			}else if(m_type == "line"){
				ax += ax_ref;
				ay += ay_ref;
				bx += ax_ref;
				by += ay_ref;

		    	l_objectifs.push_back(new_mission_lf(ax, ay, bx, by, -2));
			}else if(m_type == "circle"){
				l_objectifs.push_back(new_mission_circle(ax, ay, bx, by, 3., 1.));
			}else if(m_type == "arc"){
				ax += ax_ref;
				ay += ay_ref;
				l_objectifs.push_back(new_mission_arc(ax, ay, by, bx, r));
				//l_objectifs.push_back(new_mission_arc(ax, ay, bx, by, 3., 1., cap_sortie*M_PI/180.));

			}

			ROS_INFO("Lecture d'une mission (m_type : %s)(%f, %f, %f, %f, %f)", m_type.c_str(), ax, ay, bx, by, r);

		}
	}
	/**/
	//A : point de départ

 	//float lat0 = 48.418197;//48.418124; 
 	//float lon0 = -4.473515;//-4.473599;
 
 	//B
 	//float lat1 = 48.418790;//48.418079; 
 	//float lon1 = -4.473591;//-4.474119;

 	//C
 	//float lat2 = 48.418747;//48.418857; 
 	//float lon2 = -4.474333;//-4.474267;
  
  	//D
 	//float lat3 = 48.418131;//48.418900; 
 	//float lon3 = -4.474223;//-4.473706;

	//float lat0 = 48.418635; 
 	//float lon0 = -4.473634;
 
 	//float lat1 = 48.418624; 
 	//float lon1 = -4.474211;

 	//float lat2 = 48.418857; 
 	//float lon2 = -4.474267;
  
 	//float lat3 = 48.418900; 
 	//float lon3 = -4.473706;

 	float lat0 = 48.418197; 
 	float lon0 = -4.473515;
 
 	float lat1 = 48.418790; 
 	float lon1 = -4.473591;

 	float lat2 = 48.418747; 
 	float lon2 = -4.474333;
  
 	float lat3 = 48.418131; 
 	float lon3 = -4.474223;


 	float ax,ay,bx,by,cx,cy,dx,dy;
 	int nb_rails = 10;
	r_offset = 8.;
	LatLonToUTMXY(lat0,lon0,0,ay,ax);
    LatLonToUTMXY(lat1,lon1,0,by,bx);
    LatLonToUTMXY(lat2,lon2,0,cy,cx);
    LatLonToUTMXY(lat3,lon3,0,dy,dx);

    //std::cout << ax << " | " << ay << " | " << bx << " | " << by << std::endl;
    //std::cout << cx << " | " << cy << " | " << dx << " | " << dy << std::endl;

    std::vector<std::pair<float,float>> ligne_0;
    std::vector<std::pair<float,float>> ligne_1;

    std::pair<float,float> vect_0 = {(dx-ax)/(2.*(float)nb_rails),(dy-ay)/(2.*(float)nb_rails)};
    std::pair<float,float> vect_1 = {(cx-bx)/(2.*(float)nb_rails),(cy-by)/(2.*(float)nb_rails)};

    for(int i = 0; i < 2*nb_rails ;i++){
	    std::pair<float,float> p0;
	    std::pair<float,float> p1;
	    
	    p0.first = ax + vect_0.first*i;
		p0.second = ay + vect_0.second*i;
	    p1.first = bx + vect_1.first*i;
		p1.second = by + vect_1.second*i;

		ligne_0.push_back(p0);
		ligne_1.push_back(p1);
    }


    for(int i = 0; i < ligne_1.size() - 3; i++){
    	//new_mission_circle(float ax, float ay, float bx, float by, float r_offset, int sens)
    	//new_mission_lf(float ax, float ay, float bx, float by, float offset_validation)
    	l_objectifs.push_back(new_mission_lf(ligne_0[i].first, ligne_0[i].second, ligne_1[i].first, ligne_1[i].second, 0));
    	l_objectifs.push_back(new_mission_circle(ligne_1[i].first, ligne_1[i].second, ligne_1[i+3].first, ligne_1[i+3].second, r_offset, -1));
    	l_objectifs.push_back(new_mission_lf(ligne_1[i+3].first, ligne_1[i+3].second, ligne_0[i+3].first, ligne_0[i+3].second, 0));
    	l_objectifs.push_back(new_mission_circle(ligne_0[i+3].first, ligne_0[i+3].second, ligne_0[i+1].first, ligne_0[i+1].second, r_offset, -1));
    	
   	}
   	
   	
   	ROS_INFO("Mission Node : Fin de la lecture des missions, %i objectifs a accomplir", l_objectifs.size());
}

void sendMissions(){
	//affichage des buts de foot

	//A
	float lat0 = 48.418924; 
 	float lon0 = -4.473937;
 
 	//B
 	float lat1 = 48.418920; 
 	float lon1 = -4.474019;

 	 	//C
 	float lat2 = 48.418031; 
 	float lon2 = -4.473807;
  
  	//D
 	float lat3 = 48.418025;
 	float lon3 = -4.473887;

 	float ax,ay,bx,by,cx,cy,dx,dy;

	LatLonToUTMXY(lat0,lon0,0,ay,ax);
    LatLonToUTMXY(lat1,lon1,0,by,bx);
    LatLonToUTMXY(lat2,lon2,0,cy,cx);
    LatLonToUTMXY(lat3,lon3,0,dy,dx);

	regul::msg_mission msn;	
	msn.type = 1;
	msn.x0 = ax;
	msn.y0 = ay;
	msn.x1 = bx;
	msn.y1 = by;
	chatter_draw.publish(msn);

	msn.type = 1;
	msn.x0 = cx;
	msn.y0 = cy;
	msn.x1 = dx;
	msn.y1 = dy;
	chatter_draw.publish(msn);

	regul::msg_mission msn2;
	//utiliser pour l'affichage de l'interface graphique
	for(int i = 0; i < l_objectifs.size(); i++){
		regul::msg_mission msn;
		if(l_objectifs[i].type == 3){
			msn.type = 2;

			//draw le point de sortie
			msn2.type = 0;
			msn2.x0 = l_objectifs[i].p0x;
			msn2.y0 = l_objectifs[i].p0y;
			msn2.x1 = l_objectifs[i].p0x +l_objectifs[i].r*cos(l_objectifs[i].p1x);
			msn2.y1 = l_objectifs[i].p0y +l_objectifs[i].r*sin(l_objectifs[i].p1x);
			chatter_draw.publish(msn2);
		}else{
			msn.type = l_objectifs[i].type;
		}
		msn.x0 = l_objectifs[i].p0x;
		msn.y0 = l_objectifs[i].p0y;
		msn.x1 = l_objectifs[i].p1x;
		msn.y1 = l_objectifs[i].p1y;
		msn.r = l_objectifs[i].r;
		chatter_draw.publish(msn);
	}
}

void callbackAsk(const std_msgs::Bool &msg){
	//ROS_INFO("Mission Node : ask for objectives to draw");
	sendMissions();
}

mission loadMission(){
	current_mission_id++;
	mission current_mission = l_objectifs[current_mission_id];
	current_mission_type = current_mission.type;
	bool send = false;
	if(current_mission_id<l_objectifs.size()){
		if(current_mission_type == 0)
		{
			ROS_INFO("Mission Node : Debut d'une mission (numero : %i) de suivi de waypoint.",current_mission_id);
			send = true;
		}
		else if(current_mission_type == 1)
		{
			ROS_INFO("Mission Node : Debut d'une mission (numero : %i) de suivi de ligne.",current_mission_id);
			send = true;
		}
		else if(current_mission_type == 2 || current_mission_type == 3)
		{
			ROS_INFO("Mission Node : Debut d'une mission (numero : %i) de suivi de cercle.",current_mission_id);
			send = true;
		}
		else
		{
			//si le mission type n'est pas reconnu, on passe à la mission suivante
			loadMission();
		}
	}

	

	return current_mission;
	
}

bool testEndMission(mission &current_mission){
	/**
	On regarde si la condition de fin de mission est vrai
	Si on arrête, renvoie true
	sinon, envoie false
	**/
	current_mission_type = current_mission.type;

	if(current_mission_type == 0) //Waypoint following
	{
		float dist_b = dist_to_b(current_mission.p0x,current_mission.p0y,current_mission.p1x,current_mission.p1y);
	    //ROS_INFO("%f",dist_b);
		if(dist_b > current_mission.offset_validation and estim_correct and dist_b < 1000)
		{
			ROS_INFO("Mission Node : Fin du suivi de waypoint");
			return true;
		}

	} else if(current_mission_type == 1) //Line following
    {
	    float dist_b = dist_to_b(current_mission.p0x,current_mission.p0y,current_mission.p1x,current_mission.p1y);
	    //ROS_INFO("Mission Node : line follow dist_b = %f",dist_b);
	    
		if(dist_b > current_mission.offset_validation  and estim_correct and dist_b < 1000)
		{
			ROS_INFO("Mission Node : Valide à %f m",dist_b);
			ROS_INFO("Mission Node : Fin du suivi de Line");
			return true;
		}
	}else if(current_mission_type == 2) //Circle following
	{
		float dist_b = dist_to_b(l_objectifs[current_mission_id-1].p1x,l_objectifs[current_mission_id-1].p1y,l_objectifs[current_mission_id+1].p0x,l_objectifs[current_mission_id+1].p0y);
		if(cond_angul == false)
		{
			float angle = atan2(l_objectifs[current_mission_id+1].p1y-l_objectifs[current_mission_id+1].p0y,l_objectifs[current_mission_id+1].p1x-l_objectifs[current_mission_id+1].p0x);
			ROS_INFO("%f %f deg %f deg",dist_b, estim_theta*180./M_PI, angle*180./M_PI);
			if(fabs(sawtooth(estim_theta - angle)) < 0.2)
			{
				cond_angul = true;
				ROS_INFO("Mission Node : Condition angulaire validee");
			}
		}
		if(dist_b > current_mission.offset_validation and cond_angul){
			cond_angul = false;
			return true;
		} 

	}else if(current_mission_type == 3) //arc following
	{
		float dist_b = dist_to_b(l_objectifs[current_mission_id-1].p1x,l_objectifs[current_mission_id-1].p1y,l_objectifs[current_mission_id+1].p0x,l_objectifs[current_mission_id+1].p0y);
		if(cond_angul == false)
		{
			float angle = current_mission.p1x; //p1x=cap-sortie pour la mission arc
			ROS_INFO("%f %f deg %f deg",dist_b, estim_theta*180./M_PI, angle*180./M_PI);
			if(fabs(sawtooth(estim_theta - angle)) < 0.2)
			{
				cond_angul = true;
				ROS_INFO("Mission Node : Condition angulaire validee");
			}
		}
		if(dist_b > current_mission.offset_validation and cond_angul){
			cond_angul = false;
			return true;
		} 

	}
	return false;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "read");
	ros::NodeHandle n;

 	chatter_mission = n.advertise<regul::msg_mission>("/currentMission", 1000);
 	ros::Subscriber sub_ask = n.subscribe("/ask_points", 1000, callbackAsk);
 	chatter_draw = n.advertise<regul::msg_mission>("/draw_wp", 1000);
	ros::Subscriber sub_estim = n.subscribe("/estim", 1000, callbackEstim);
	ros::Publisher pub_emergency = n.advertise<std_msgs::Bool>("/emergency", 1000);

 	ros::Rate loop_rate(10);

 	bool continue_missions=true;

 	//read("/home/bertrand/workspaceRosSaturneSimu/fichier.txt");
 	read("/home/saturne/Robot_4_roues/data_traj.txt");

 	current_mission_id = -1;
 	mission current_mission = loadMission();
 	bool send = true;
    while (ros::ok()){
    	
		continue_missions = testEndMission(current_mission);
		if(continue_missions){
			current_mission = loadMission();
		}
		if(current_mission_id>=l_objectifs.size()){
			ROS_INFO("Mission Node : Fin des missions");
			std_msgs::Bool b;
    		b.data = true;
    		pub_emergency.publish(b);
			break;
		}
		if(send){
			//ROS_INFO("Mission Node : Send new objectives");
			regul::msg_mission msn;
			msn.type = current_mission.type;
			msn.x0 = current_mission.p0x;
			msn.y0 = current_mission.p0y;
			msn.x1 = current_mission.p1x;
			msn.y1 = current_mission.p1y;
			msn.r = current_mission.r;
			//msn.sens = current_mission.sens;

			chatter_mission.publish(msn);
		}

	loop_rate.sleep();
	ros::spinOnce();
	}

	return 0;
}
