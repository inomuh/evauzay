#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "im_msgs/WheelVel.h"
#include <geometry_msgs/Twist.h> 
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"

#include <stdlib.h> 

#define WHEEL_SEPERATION 0.32 // m

class evarobot_vel
{
public:
  evarobot_vel();
  void sonar0Read(const sensor_msgs::Range::ConstPtr& msg);
  void sonar1Read(const sensor_msgs::Range::ConstPtr& msg);
  void sonar2Read(const sensor_msgs::Range::ConstPtr& msg);
  void sonar3Read(const sensor_msgs::Range::ConstPtr& msg);
  void cmdRead(const geometry_msgs::Twist::ConstPtr& msg);
  void callbackSondajStatus(const std_msgs::Int32::ConstPtr& msg);
  void publishWheelVel();


private:
  ros::NodeHandle n;
  ros::Publisher pub_vel;
  ros::Publisher pub_hareketStatus;
  
  ros::Subscriber subSonar0;
  ros::Subscriber subSonar1;
  ros::Subscriber subSonar2;
  ros::Subscriber subSonar3;
  
  ros::Subscriber subSondaj;
  ros::Subscriber sub_vel;
  
  float linVel;
  float angVel;
  int sondajStatus;

  double carpismaEsik;
  
  bool carpismaDurumu[4];

};//End of class 
   
   
   
evarobot_vel::evarobot_vel()
  {
    //Topic you want to publish
    pub_vel = n.advertise<im_msgs::WheelVel>("/cntr_wheel_vel", 10);
    pub_hareketStatus = n.advertise<std_msgs::Int8>("/hareketStatus",10);

    //Topic you want to subscribe
    sub_vel = n.subscribe("/cmd_vel", 2, &evarobot_vel::cmdRead, this);
    subSonar0 = n.subscribe("/sonar0", 2, &evarobot_vel::sonar0Read, this);
    subSonar1 = n.subscribe("/sonar1", 2, &evarobot_vel::sonar1Read, this);
    subSonar2 = n.subscribe("/sonar2", 2, &evarobot_vel::sonar2Read, this);
    subSonar3 = n.subscribe("/sonar3", 2, &evarobot_vel::sonar3Read, this);

    subSondaj = n.subscribe("/sondajStatus", 2, &evarobot_vel::callbackSondajStatus, this);
    
    n.param<double>("evarobot_vel/carpismaEsik", carpismaEsik, 0.50);

    carpismaDurumu[0] = false;
    carpismaDurumu[1] = false;
    carpismaDurumu[2] = false;
    carpismaDurumu[3] = false;

    linVel = 0.0;
    angVel = 0.0;

    sondajStatus = 0.0;
    
  }
  

void evarobot_vel::callbackSondajStatus(const std_msgs::Int32::ConstPtr& msg) {
  sondajStatus = msg->data;
}

void evarobot_vel::cmdRead(const geometry_msgs::Twist::ConstPtr& msg)
{
	linVel = msg->linear.x;
	angVel = msg->angular.z;

	ROS_INFO("linVel: %f, angVel: %f", linVel, angVel);
}
  
void evarobot_vel::sonar0Read(const sensor_msgs::Range::ConstPtr& msg)
{
  if (msg->range < carpismaEsik)
  {
	this->carpismaDurumu[0] = true;
  }
  else
  {
	this->carpismaDurumu[0] = false;
  }
}

void evarobot_vel::sonar1Read(const sensor_msgs::Range::ConstPtr& msg)
{
  if (msg->range < carpismaEsik)
  {
        this->carpismaDurumu[1] = true;
  }
  else
  {
        this->carpismaDurumu[1] = false;
  }

}

void evarobot_vel::sonar2Read(const sensor_msgs::Range::ConstPtr& msg)
{
  if (msg->range < carpismaEsik)
  {
        this->carpismaDurumu[2] = true;
  }
  else
  {
        this->carpismaDurumu[2] = false;
  }

}

void evarobot_vel::sonar3Read(const sensor_msgs::Range::ConstPtr& msg)
{
  if (msg->range < carpismaEsik)
  {
        this->carpismaDurumu[3] = true;
  }
  else
  {
        this->carpismaDurumu[3] = false;
  }

}

void evarobot_vel::publishWheelVel()
{
	im_msgs::WheelVel msg;
	std_msgs::Int8 msgStatus;

	n.param<double>("evarobot_vel/carpismaEsik", carpismaEsik, 0.50);

	if (this->sondajStatus >= 8)
	{
                  // Carpisma robotu Durdur.
                  ROS_INFO("Sondaj Acik");

                  msg.right_vel = 0.0;
                  msg.left_vel = 0.0;
		  msgStatus.data = 2;
	
	}
	else if( (this->carpismaDurumu[0] || this->carpismaDurumu[1]) && this->linVel > 0 )
	{
		  // Carpisma robotu Durdur.
		  ROS_INFO("İleri Carpisma Durumu");

		  msg.right_vel = 0.0;
		  msg.left_vel = 0.0;
                  msgStatus.data = 3;
		  
	}
	else if((this->carpismaDurumu[2] || this->carpismaDurumu[3]) && this->linVel < 0 )
        {
                  // Carpisma robotu Durdur.
                  ROS_INFO("Geri Carpisma Durumu");

                  msg.right_vel = 0.0;
                  msg.left_vel = 0.0;
		  msgStatus.data = 4;

        }
        else if((this->carpismaDurumu[0] || this->carpismaDurumu[3]) && this->angVel > 0 )
        {
                  // Carpisma robotu Durdur.
                  ROS_INFO("Geri Carpisma Durumu");

                  msg.right_vel = 0.0;
                  msg.left_vel = 0.0;
                  msgStatus.data = 5;

        }
        else if((this->carpismaDurumu[1] || this->carpismaDurumu[2]) && this->angVel < 0 )
        {
                  // Carpisma robotu Durdur.
                  ROS_INFO("Geri Carpisma Durumu");

                  msg.right_vel = 0.0;
                  msg.left_vel = 0.0;
                  msgStatus.data = 6;

        }
	else
	{
          	 if(fabs(this->linVel)<=0.01 && fabs(this->angVel)<= 0.01)
	  	 {
			msgStatus.data = 0;
		 }
		 else
		 {
			msgStatus.data = 1;
		 }
	 	 // Normal hızı uygula
	   	 msg.left_vel = 0.5*(2.0 * this->linVel
                           - this->angVel * WHEEL_SEPERATION);  // m/s
		 msg.right_vel = 0.5*(2.0 * this->linVel
                           + this->angVel * WHEEL_SEPERATION);  // m/s

	  ROS_INFO("leftVel: %f, rightVel: %f", msg.left_vel, msg.right_vel);

	}
	
	pub_vel.publish(msg);
        pub_hareketStatus.publish(msgStatus);
	
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "evarobot_vel");

  //Sets up the random number generator
  //srand(time(0));

  evarobot_vel velObject;
  
  //Sets the loop to publish at a rate of 10Hz
  ros::Rate rate(10);
  while(ros::ok())
  {

	velObject.publishWheelVel();
	  
	  ros::spinOnce();
	  rate.sleep();
  }
  return 0;
}
