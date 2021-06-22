#include "compass.h"

//https://fr.m.wikipedia.org/wiki/Fichier:Simple_compass_rose-fr.svg
compass::compass(QWidget* parent) : QLabel(parent)
{
	size = 300;
	std::string path = ros::package::getPath("ihm");
	setPixmap(QPixmap(QString::fromUtf8(path.c_str())+"/compass.png").scaled(size,size));
	heading = 0;
	heading_obj = M_PI;

}

compass::~compass()
{

}

void compass::setHeading(float heading_)
{
	heading = heading_;
	update();
}

void compass::paintEvent(QPaintEvent *e)
{
	QLabel::paintEvent(e);
    QPainter painter(this);
    painter.setPen(QPen(Qt::red, 4));
    painter.drawLine(size/2.,size/2.,size/2.+0.8*size/2.*cos(heading-M_PI/2),size/2.+0.8*size/2.*sin(heading-M_PI/2));
    //painter.setPen(QPen(Qt::blue, 4));
    //painter.drawLine(size/2.,size/2.,size/2.+0.8*size/2.*cos(heading_obj-M_PI/2),size/2.+0.8*size/2.*sin(heading_obj-M_PI/2));
}