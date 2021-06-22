#include "ihm.h"

double max_cov = 2.5;


void ihm::callbackDraw(const regul::msg_mission& msg)
{
	send = false;
	if(msg.type == 0)
	{
		map_->addWp(msg.x0,msg.y0,msg.x1,msg.y1);
	}
	else if(msg.type == 2)
	{
		map_->addCircle(msg.x0,msg.y0,msg.r);
	}
	else
	{
		map_->addLine(msg.x0,msg.y0,msg.x1,msg.y1);
	}
}

void ihm::callbackFix(const sensor_msgs::NavSatFix& msg)
{

	if(msg.position_covariance[0] <= max_cov and msg.position_covariance[4] <= max_cov and !isnan(msg.latitude) and !isnan(msg.longitude))
	{
		map_->addPoint(msg.latitude,msg.longitude,current_heading);
	}
	label_lat->setText(QString::number(msg.latitude));
	label_lon->setText(QString::number(msg.longitude));
	label_cov->setText("("+QString::number(msg.position_covariance[0]) +","+ QString::number(msg.position_covariance[4])+")");

}

void ihm::callbackEstim(const regul::msg_estim& msg)
{

	if(msg.correct == true)
	{
		map_->addPointEstim(msg.x,msg.y,-msg.theta);
	}
}

void ihm::callbackImu(const sensor_msgs::Imu& msg)
{
	current_heading = -tf::getYaw(msg.orientation);
	compass_->setHeading(-current_heading);
	label_heading->setText(QString::number(-current_heading*180./M_PI)+"°");
}

void ihm::callback(const ros::MessageEvent<std_msgs::Float64 const>& event)
{ 
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  const std_msgs::Float64ConstPtr& msg = event.getMessage();
  for(int i = 0; i < data_list.size(); i++)
  {
	  if(data_list[i].topic == topic)
	  {
		  data_list[i].data->setText(QString::number(msg->data)+data_list[i].suffix);
		  break;
	  }
  }
  
}

ihm::ihm()
{
	static ros::NodeHandle n;
	std::string path = ros::package::getPath("ihm");
	settings = new QSettings(QString::fromUtf8(path.c_str())+"/conf.ini",QSettings::IniFormat);

	int nb_data = settings->value("nb_data").toInt();
	widget_central = new QWidget;
	layout_central = new QHBoxLayout;

	compass_ = new compass(this);
	temp = new QTextEdit;

	scroll = new QScrollArea;
	scroll->setWidgetResizable(false);
	map_ = new map(this,scroll,settings->value("map/lat0").toFloat(),settings->value("map/lon0").toFloat(),settings->value("map/lat4").toFloat(),settings->value("map/lon4").toFloat());
	
	layout_infos = new QVBoxLayout;
	etat = new QGroupBox("Information");
	layout_etat = new QFormLayout;
	timer = new QTimer;
	for(int i = 0; i < nb_data; i++)
	{
		data_display info;
		QLabel *label_data = new QLabel("");
		layout_etat->addRow(settings->value("data_"+QString::number(i)+"/name").toString(),label_data);
		info.data = label_data;
		info.suffix = settings->value("data_"+QString::number(i)+"/suffix").toString();
		info.topic  = settings->value("data_"+QString::number(i)+"/topic").toString().toUtf8().constData();
		info.sub = n.subscribe(info.topic, 1000, &ihm::callback,this);
		data_list << info;
		
	}


	state = new QGroupBox("Position");
	layout_state = new QFormLayout;
	label_lat = new QLabel("");
	label_lon = new QLabel("");
	label_cov = new QLabel("");
	label_heading = new QLabel("");
	sub_fix = n.subscribe("/fix", 1000, &ihm::callbackFix,this);
	sub_estim = n.subscribe("/estim", 1000, &ihm::callbackEstim,this);
	sub_imu = n.subscribe("/imu/data", 1000, &ihm::callbackImu,this);
	sub_draw = n.subscribe("/draw_wp", 1000, &ihm::callbackDraw,this);
	chatter_ask = n.advertise<std_msgs::Bool>("/ask_points", 1000);

	layout_state->addRow("Latitude:",label_lat);
	layout_state->addRow("Longitude:",label_lon);
	layout_state->addRow("Covariance:",label_cov);
	layout_state->addRow("Heading:",label_heading);
	state->setLayout(layout_state);




	etat->setLayout(layout_etat);

	layout_infos->addWidget(compass_);
	layout_infos->addWidget(state);
	layout_infos->addWidget(etat);
	layout_infos->addWidget(temp);

	layout_central->addWidget(scroll);
	scroll->setWidget(map_);
	layout_central->addLayout(layout_infos);
	widget_central->setLayout(layout_central);
	setCentralWidget(widget_central);

	setWindowTitle("Robot control");
	timer->start(50);
	send = true;

	connect(timer, SIGNAL(timeout()),this, SLOT(rosspin()));
}


ihm::~ihm()
{



}


void ihm::rosspin()
{
	timer->start(50);
	if(!ros::ok())
	{
		qApp->quit();
	}
	else
	{
		ros::spinOnce();
	}
	if(send)
	{
		std_msgs::Bool msg;
		msg.data = true;
		chatter_ask.publish(msg);
	}
}