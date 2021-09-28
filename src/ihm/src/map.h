#ifndef HEADER_MAP
#define HEADER_MAP

#include <QtWidgets>
#include <QtCore>
#include <ros/package.h>
#include <string>
#include <cmath>
#include "UTM.h"

struct point{
	float x;
	float y;
	float heading;
	bool estim;
};


struct line{
	float x0;
	float y0;
	float x1;
	float y1;
};

struct circle{
	float x;
	float y;
	float r;
};



class map : public QLabel
{
	Q_OBJECT

		public:
			explicit map(QWidget* parent,QScrollArea *scroll_,float lat0,float lon0,float lat4,float lon4);
			~map();		
			void addPoint(float lat,float lon,float heading);
			void addPointEstim(float x,float y,float heading);
			void addWp(float x0,float y0,float x1,float y1);
			void addCircle(float x,float y,float r);
			void addLine(float x0,float y0,float x1,float y1);
			void setHeading(float heading);

		protected:
			void paintEvent(QPaintEvent *e);
			void wheelEvent(QWheelEvent *event);

		private:
			float x0;
			float y0;
			float x4;
			float y4;
			float dx;
			float dy;
			float zoom;
			float height;
			float width;
			QPixmap background;
			QScrollArea *scroll;
			QList<point> liste_points_pos;
			QList<line> liste_points_line;
			QList<line> liste_points_wp;
			QList<circle> liste_points_circle;
};			


#endif