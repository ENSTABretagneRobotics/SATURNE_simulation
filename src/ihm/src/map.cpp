#include "map.h"

map::map(QWidget* parent,QScrollArea *scroll_,float lat0,float lon0,float lat4,float lon4) : QLabel(parent)
{
	setScaledContents(true);
	//setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	scroll = scroll_;
	std::string path = ros::package::getPath("ihm");
	background = QPixmap(QString::fromUtf8(path.c_str())+"/map.png");
	setPixmap(background);
	LatLonToUTMXY(lat0,lon0,0,y0,x0);
	LatLonToUTMXY(lat4,lon4,0,y4,x4);
	height = background.size().height();
	width = background.size().width();
	zoom = 1.;
	dx = height/(x4 - x0);
	dy = width/(y4 - y0);
}

map::~map()
{

}

void map::addWp(float x0,float y0,float x1,float y1)
{
	line line_;
	line_.x0 = x0;
	line_.y0 = y0;
	line_.x1 = x1;
	line_.y1 = y1;
	liste_points_wp << line_;
	update();
}

void map::addCircle(float x,float y,float r)
{
	circle circle_;
	circle_.x = x;
	circle_.y = y;
	circle_.r = r;
	liste_points_circle << circle_;
	update();
}

void map::addLine(float x0,float y0,float x1,float y1)
{
	line line_;
	line_.x0 = x0;
	line_.y0 = y0;
	line_.x1 = x1;
	line_.y1 = y1;
	liste_points_line << line_;
	update();

}

void map::addPoint(float lat,float lon,float heading)
{
	point pt;
	float x,y;
    LatLonToUTMXY(lat,lon,0,y,x);
	pt.x = x;
	pt.y = y;
	pt.heading = heading;
	pt.estim = true;
	liste_points_pos << pt;
	update();
}

void map::addPointEstim(float x,float y,float heading)
{
	point pt;
	pt.x = x;
	pt.y = y;
	pt.estim = false;
	pt.heading = heading;
	//liste_points_pos << pt;
	//update();
}

void map::paintEvent(QPaintEvent *e)
{
	float x,y;
	float center_lat = 48.418501;
	float center_long = -4.473898; 
	
	resize(width*zoom,height*zoom);
    QPainter painter(this);
    painter.scale(zoom,zoom);
    painter.drawPixmap(0,0,background);
    painter.setPen(QPen(Qt::blue, 2));
    for(int i = 0; i < liste_points_circle.size(); i++)
    {
    	painter.drawEllipse((liste_points_circle[i].y-y0 -liste_points_circle[i].r)*dy,(liste_points_circle[i].x-x0-liste_points_circle[i].r)*dx,liste_points_circle[i].r*2.*dy,liste_points_circle[i].r*2.*dx);
    }
    painter.setPen(QPen(Qt::black, 1));
    for(int i = 0; i < liste_points_wp.size(); i++)
    {
    	painter.drawLine((liste_points_wp[i].y0-y0)*dy,(liste_points_wp[i].x0-x0)*dx,(liste_points_wp[i].y1-y0)*dy,(liste_points_wp[i].x1-x0)*dx);
    	painter.drawEllipse((liste_points_wp[i].y1-y0)*dy-1.5,(liste_points_wp[i].x1-x0)*dx-1.5,3,3);
    }
    painter.setPen(QPen(Qt::red, 1));
    for(int i = 0; i < liste_points_line.size(); i++)
    {
    	painter.drawLine((liste_points_line[i].y0-y0)*dy,(liste_points_line[i].x0-x0)*dx,(liste_points_line[i].y1-y0)*dy,(liste_points_line[i].x1-x0)*dx);
    }
	 painter.setPen(QPen(Qt::blue, 1));
    for(int i = 0; i < liste_points_pos.size(); i++)
    {
    	if(liste_points_pos[i].estim)
    	{
    		painter.setPen(QPen(Qt::green, 1));
    	}
    	else
    	{
    		painter.setPen(QPen(Qt::magenta, 1));
    	}
	    float heading = liste_points_pos[i].heading;
	    x = liste_points_pos[i].x;
	    y = liste_points_pos[i].y;

		QPointF points[4] = {
	    QPointF((y-y0)*dy+(-10/zoom)*sin(heading),(x-x0)*dx+(-10/zoom)*cos(heading)),
	    QPointF((y-y0)*dy-(5/zoom)*cos(heading)+(10/zoom)*sin(heading),(x-x0)*dx+(10/zoom)*cos(heading)-(5/zoom)*sin(heading)),
	    QPointF((y-y0)*dy+(5/zoom)*sin(heading),(x-x0)*dx+(5/zoom)*cos(heading)),
	    QPointF((y-y0)*dy+(5/zoom)*cos(heading)+(10/zoom)*sin(heading),(x-x0)*dx+(10/zoom)*cos(heading)+(5/zoom)*sin(heading))
		};

		painter.drawConvexPolygon(points, 4);
    }

 }

void map::wheelEvent(QWheelEvent *event)
{
    if(event->angleDelta().y() > 0)
    {
        zoom -= 0.1;
    }
    else
    {
        zoom += 0.1;        
    }

    event->accept();
    
    
    update();
 
    scroll->horizontalScrollBar()->setValue(int(zoom * scroll->horizontalScrollBar()->value()+ ((zoom- 1) * scroll->horizontalScrollBar()->pageStep()/2)));
    scroll->verticalScrollBar()->setValue(int(zoom * scroll->verticalScrollBar()->value()+ ((zoom- 1) * scroll->verticalScrollBar()->pageStep()/2)));
}