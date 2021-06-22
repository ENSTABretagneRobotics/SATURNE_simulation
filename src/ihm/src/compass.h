#ifndef HEADER_COMPASS
#define HEADER_COMPASS

#include <QtWidgets>
#include <QtCore>
#include <ros/package.h>
#include <string>
#include <cmath>


class compass : public QLabel
{
	Q_OBJECT

		public:
			explicit compass(QWidget* parent);
			~compass();	
			void setHeading(float heading_);	

		protected:
			void paintEvent(QPaintEvent *e);

		private:
			float heading;	
			float heading_obj;	
			float size;
};			


#endif