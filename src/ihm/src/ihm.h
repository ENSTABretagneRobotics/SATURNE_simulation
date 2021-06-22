#ifndef HEADER_IHM
#define HEADER_IHM

#include <QtWidgets>
#include <QtCore>
#include <ros/package.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "UTM.h"
#include "tf/tf.h"
#include "compass.h"
#include "map.h"
#include "regul/msg_mission.h"
#include "regul/msg_estim.h"




struct data_display
{
	QLabel *data;
	QString suffix;
	std::string topic;
	ros::Subscriber sub;
};

class ihm : public QMainWindow
{
	Q_OBJECT

		public:
			ihm();
			~ihm();
			void callback(const ros::MessageEvent<std_msgs::Float64 const>& event);
			void callbackFix(const sensor_msgs::NavSatFix& msg);
			void callbackImu(const sensor_msgs::Imu& msg);
			void callbackDraw(const regul::msg_mission& msg);
			void callbackEstim(const regul::msg_estim& msg);
			QTextEdit *temp;			
		private slots:
			void rosspin();


		private:
			QSettings *settings;
			QTimer *timer;
			QScrollArea *scroll;
			QWidget *widget_central;
			QVBoxLayout *layout_infos;
			QHBoxLayout *layout_central;
			QGroupBox *etat;
			QFormLayout *layout_etat;

			QGroupBox *state;
			QFormLayout *layout_state;

			QLabel *label_lat;
			QLabel *label_lon;
			QLabel *label_cov;
			QLabel *label_heading;

			ros::Subscriber sub_fix;
			ros::Subscriber sub_imu;
			ros::Subscriber sub_draw;
			ros::Subscriber sub_estim;
			ros::Publisher chatter_ask;

			QList<data_display> data_list;
			float current_heading;
			bool send;

			compass *compass_;
			map *map_;





					
};			


#endif