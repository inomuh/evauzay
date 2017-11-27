#include "evarobot_eio/evarobot_eio.h"

#include "std_srvs/Empty.h"
#include "std_msgs/Int32.h"
#include "im_msgs/SetRGB.h"

int i_error_code = 0;

// 0: Dur, 1: Başlat, 2: Bitir
int sondajDurum = 2;

ros::Time begin;

void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if(i_error_code<0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%s", GetErrorDescription(i_error_code).c_str());
        i_error_code = 0;
    }
    else
    {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EvarobotEIO: No problem.");
    }
}

bool callbackSondajBaslat(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
  begin = ros::Time::now();
  sondajDurum = 1;
  return true;
}

bool callbackSondajBitir(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
  begin = ros::Time::now();
  sondajDurum = 2;
  return true;
}

bool callbackSondajDurdur(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
  sondajDurum = 0;
  return true;
}



int main(int argc, char **argv)
{
	
	key_t key;
	sem_t *mutex;
	FILE * fd;

	key = 1005;

	mutex = sem_open(SEM_NAME,O_CREAT,0644,1);
	if(mutex == SEM_FAILED)
	{
		ROS_INFO(GetErrorDescription(-98).c_str());
        i_error_code = -98;
		sem_unlink(SEM_NAME);

		return(-1);
	}


	// ROS PARAMS
	double d_frequency;
	
	string str_i2c_path;
	// rosparams end
	
	
	
		
	ros::init(argc, argv, "evarobot_eio");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_eio/i2c_path", str_i2c_path, "/dev/i2c-1");
	
	if(!n.getParam("evarobot_eio/frequency", d_frequency))
	{
		ROS_INFO(GetErrorDescription(-99).c_str());
        i_error_code = -99;
	} 

	
	ros::ServiceServer srvBaslat = n.advertiseService("SondajBaslat", callbackSondajBaslat);
	ros::ServiceServer srvBitir = n.advertiseService("SondajBitir", callbackSondajBitir);
	ros::ServiceServer srvDurdur = n.advertiseService("SondajDurdur", callbackSondajDurdur);

	ros::Publisher pub = n.advertise<std_msgs::Int32>("sondajStatus", 10);

	ros::ServiceClient client = n.serviceClient<im_msgs::SetRGB>("evarobot_rgb/SetRGB");

	// Define frequency
	ros::Rate loop_rate(d_frequency);
	
	IMEIO * eio ;
	try{
		eio = new IMEIO(0b00100000, string("/dev/i2c-1"),  mutex);
		
		eio->SetPinBDirection(IMEIO::EIO0, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO1, IMEIO::OUTPUT);

	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
        i_error_code = e;
	}
     
	bool b_data = false;
	
	
	// ROS PARAMS
	double d_min_freq = 0.2;
	double d_max_freq = 10.0;
	std_msgs::Int32 msg;
	msg.data = 0;

	// Diagnostics
	diagnostic_updater::Updater updater;
	updater.setHardwareID("EvarobotEIO");
	updater.add("eio", &ProduceDiagnostics);

	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("eio", updater,
         diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));

	double gecenZaman = 0.0;

	im_msgs::SetRGB rgbSondajBaslat, rgbSondajBitir, rgbSondajDurdur;


	rgbSondajBaslat.request.times = -1;
	rgbSondajBaslat.request.mode = 6;
	rgbSondajBaslat.request.frequency = -1.0;
	rgbSondajBaslat.request.color = 0;

        rgbSondajBitir.request.times = -1;
        rgbSondajBitir.request.mode = 3;
        rgbSondajBitir.request.frequency = -1.0;
        rgbSondajBitir.request.color = 0;

        rgbSondajDurdur.request.times = -1;
        rgbSondajDurdur.request.mode = 0;
        rgbSondajDurdur.request.frequency = -1.0;
        rgbSondajDurdur.request.color = 0;

            
	begin = ros::Time::now();

	while(ros::ok())
	{		

	  if(gecenZaman >= 20.0) {
	    sondajDurum = 0;
	    gecenZaman = 0.0;
	  }

	  		
	  if(sondajDurum == 1) {
	    try{
	      if(msg.data < 20) {
		msg.data++;
		client.call(rgbSondajBaslat);
	      }
	      eio->SetPinBValue(IMEIO::EIO0, IMEIO::HIGH);
	      eio->SetPinBValue(IMEIO::EIO1, IMEIO::LOW);
	  
	      gecenZaman = ros::Time::now().toSec() - begin.toSec();

	    }catch(int e){
	      ROS_INFO(GetErrorDescription(e).c_str());
	      i_error_code = e;
	    }

	  } else if(sondajDurum == 2) {
	    try{
	      
	      if(msg.data > 0) {
		msg.data--;
		client.call(rgbSondajBitir);
	      }
              
	      eio->SetPinBValue(IMEIO::EIO0, IMEIO::LOW);
              eio->SetPinBValue(IMEIO::EIO1, IMEIO::HIGH);
	  
	      gecenZaman = ros::Time::now().toSec() - begin.toSec();

            }catch(int e){
              ROS_INFO(GetErrorDescription(e).c_str());
              i_error_code = e;
	    } 
	  } else {
	    try{
              eio->SetPinBValue(IMEIO::EIO0, IMEIO::LOW);
              eio->SetPinBValue(IMEIO::EIO1, IMEIO::LOW);
	      client.call(rgbSondajDurdur);

            }catch(int e){
              ROS_INFO(GetErrorDescription(e).c_str());
              i_error_code = e;
            }

	  }

	  pub.publish(msg);
	  ros::spinOnce();
	  updater.update();
	  loop_rate.sleep();	
	
	}
	
	
	return 0;
}
