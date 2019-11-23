#include <ros/ros.h>


#include <stdio.h>
#include <math.h>

#include "mavros_msgs/PositionTarget.h" //Mavros Topic to publish 
#include "drone_controller/states.h" //Custom Msg to recive KF states
#include "drone_controller/error.h"
#include <mavros_msgs/CommandTOL.h> //Service for landing


#define FACTORX  0,00234375 //0.003125, this Factor doesnt saturate the system
#define FACTORY  0.003125 //0.003125, this Factor doesnt saturate the system
#define FACTORTH  0.0055 //Rotation factor
#define FACTORZ  0.05 //Descend Factor

#define MAXV  0.75 //Max Vx and Vy speed
#define MINV -0.75 //Min Vx and Vy speed
#define MAXR  0.5 //Max Yaw rate
#define MINR -0.5 //Min Yaw rate


class Controller //Controller class
{
  private: //Private class atributes

  ros::NodeHandle po_nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher pub1;
  ros::Time lastTime;
  ros::ServiceClient land_client;

  double imageW; //Image Width
  double imageH; //Image Height
  float zini; //Initial Z pos

  public: //Public clas atributes 
  Controller(ros::NodeHandle ao_nh) : po_nh(ao_nh) //Constructor of the clas Controller
  {
    pub = po_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10) ; //Publisher to dron velocities, Z pos and Yaw
    pub1 = po_nh.advertise<drone_controller::error>("/error",10) ;
    sub = po_nh.subscribe("/predicted_states", 10, &Controller::controllerCallBack, this); ////Predicted states subscriber (KF states)
    lastTime = ros::Time::now(); //ROS time initialization throw ros time
    imageW = 640/2;  //Image W size
    imageH = 480/2;  //Image H size
  }

  void controllerCallBack(const drone_controller::states& msg) //Callback para el subscriber
  {

	//Image data attributes
	double centX = (double) msg.Xc; //Parsing float into double
	double centY = (double) msg.Yc;
	double width = (double) msg.W;
	double heigh = (double) msg.H;
	double theta = (double) msg.Theta;

	// Time since last call
        double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec(); //The average publish time is of 10ms
        lastTime = ros::Time::now();

        //Error Calculation between image and template's center
	float ErX = imageW - centX; //Error in X of the image
        float ErY = imageH - centY; //Error in Y of the image
        float ErTheta = theta; //Error in Theta of the image

	//Variables to be published (Initialized in 0)
	float vx = 0;
	float vy = 0;
	float vthe = 0;



	// Calculate Vx. Vy and Yaw rate
        vx = -1 * ErX * FACTORX; //Ex for a multiplication factor
        vy = ErY * FACTORY; //Ey for a multiplication factor
	vthe = ErTheta * FACTORTH; //Etheta for a multiplication factor 

	if(ErX < 3 && ErY < 3)
	{
	vthe = -1 * ErTheta * FACTORTH;
	}
	
	// Limit the Vx
        if (vx > MAXV)
        {
            vx = MAXV;
        } else if (vx < MINV)
        {
            vx = MINV;
        }

        // Limit the Vy
        if (vy > MAXV)
        {
            vy = MAXV;
        } else if (vy < MINV)
        {
            vy = MINV;
        }

	// Limit the Yaw rate
        if (vthe > MAXR)
        {
            vthe = MAXR;
        } else if (vthe < MINR)
        {
            vthe = MINR;
        }

	
	//Pos object
        mavros_msgs::PositionTarget pos; //Creation of the setpoint_raw/local object
	
	pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;  //FRAME_LOCAL_NED to move the drone frame
  
        pos.header.stamp = ros::Time::now(); //Time header stamp
        pos.header.frame_id = "base_link"; //"base_link" frame to compute odom
        pos.type_mask = 1987; //Mask for Vx, Vy, Z pos and Yaw rate
        pos.position.z = 2.25;
        pos.velocity.x = vx;//X(+) Drone left, X(-) Drone Right
        pos.velocity.y = vy;//Y(+) Drone Backward, X(-) Drone Forward
        pos.yaw_rate = vthe;//////Yaw(+) Drone Anti horario, Yaw(-) Drone Horario

	printf("Dany Vx, Vy, Vthe  values at (%f,%f,%f) \n", vx, vy, vthe);
	pub.publish(pos);

        drone_controller::error data;
	data.errorX = ErX;
	data.errorY = ErY;
	data.errorT = ErTheta;
	data.errorS = 0;
	
	printf("Error at Vx, Vy and Theta (%f,%f,%f) \n", ErX, ErY, ErTheta);
	pub1.publish(data);
  }


};//End of class 


int main(int argc, char** argv)
{   

    ros::init(argc, argv, "trejos_controller_node"); //Node name
    ros::NodeHandle n;
    Controller cont(n);


    ros::spin();

    return 0;
}
