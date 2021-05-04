#include <ros/ros.h>
#include "ihm.h"
#include <QApplication>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Ihm_node");
	QApplication app(argc, argv);
	ihm fen;
	
	fen.show();
	app.exec();
    return 0;
}