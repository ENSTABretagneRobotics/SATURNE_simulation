#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cmath>
#include "geometry_msgs/Twist.h"

double rot_max = 2.5;

bool display = false;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Line_follow_node");
  ros::NodeHandle n;
  if(display)
  {
    cv::namedWindow("View");
    cv::namedWindow("Output");
    cv::startWindowThread();
  }
  
  ros::Publisher chatter_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Rate loop_rate(5);
  cv::VideoCapture cap;
  geometry_msgs::Twist msg;
  if(!cap.open(0))
  {
      ROS_ERROR("Pas de camera");   
      return 0;
  }
  while (ros::ok())
  {
    cv::Mat frame;
    cap >> frame;

    cv::rotate(frame,frame, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::GaussianBlur(frame,frame,cv::Size(5,5),0,0);
    cv::Mat hsv,mask,res,out;
    //Passage en HSV
    cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
    cv::Scalar min = cv::Scalar(90, 100,0);
    cv::Scalar max = cv::Scalar(110,255, 255);
    inRange(hsv, min, max,mask);
    cv::bitwise_and(hsv,hsv,res,mask);
    cv::cvtColor(res, out, cv::COLOR_HSV2BGR);
    if(display)
    {
      imshow("Output",out);
    }
    //out = out-(Ouverture(Ouverture(Ouverture(Fermeture(out)))));
    cv::Mat grey;
    cv::Mat bin;
    cv::cvtColor(out,grey,cv::COLOR_RGB2GRAY);
    cv::threshold(grey,bin,0,255, CV_THRESH_BINARY| CV_THRESH_OTSU);
    cv::Mat edges;
    cv::Canny(bin, edges, 50, 200);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180., 20, 120, 50);

    float x1,x2,y1,y2;
    float n = 0,sumTheta =0,sumPos=0;
    // On trouve l'abcisse du centre de la ligne et son orientation
    for (size_t i=0; i<lines.size(); i++) 
    {
        cv::Vec4i l = lines[i];
        x1 = l[0],y1 = l[1],x2 = l[2],y2 = l[3];
        float angle_line = atan2(y2-y1,x2-x1);
        cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, CV_AA);
        if (fabs(angle_line) < M_PI/3) // On filtre les lignes perpendiculaires
        {
            sumTheta += atan2(y2-y1,x2-x1);
            n += 1;
            sumPos += (y1+y2)/2 ; 
            cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, CV_AA);
        }
        else
        {
            cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA); 
        }
    }

    bool data_correct;
    double pos;
    double angle;

    if(n > 0) // On a trouvé une ligne correcte
    {
      
      pos = ((sumPos/n)/out.rows - 0.5);
      angle = sumTheta/n;
      data_correct = true;    
          
    }
    else // On n'a pas trouvé de ligne correcte
    {
      pos = 0;
      angle = 0;
      data_correct = false;
    }

    if(data_correct)
    {
      double diff = -10.*(-0.9*pos - angle/16.); //Proportionnel dérivé
      //msg.cmd_left = std::min(std::max(300.-diff,-100.),500.);
      //msg.cmd_right = std::min(std::max(300.+diff,-100.),500.);
      //msg.cmd_left = std::min(std::max(200.-diff,100.),300.);
      //msg.cmd_right = std::min(std::max(200.+diff,100.),300.);
      msg.linear.x = 0.5;
      msg.angular.z = std::min(std::max(diff,-rot_max),rot_max);
    }
    else
    {
      msg.linear.x = 0.;
      msg.angular.z = 0.;
    }
    std::cout << msg.linear.x << " " << msg.angular.z*180./M_PI << std::endl;

    chatter_cmd.publish(msg);
    if(display)
    {
      cv::imshow("View",frame );
    }
    ros::spinOnce();
  }
  if(display)
  {
    cv::destroyWindow("view");
  }
  return 0;
}
