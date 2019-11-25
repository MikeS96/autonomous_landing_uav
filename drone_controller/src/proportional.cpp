#include <ros/ros.h>


#include <stdio.h>
#include <math.h>

#include "mavros_msgs/PositionTarget.h" //Mavros Topic to publish 
#include "drone_controller/states.h" //Custom Msg to recive KF states
#include "drone_controller/error.h"
#include <mavros_msgs/CommandTOL.h> //Service for landing

#define FACTORX  0.0015625 //0.003125, this Factor doesnt saturate the system
#define FACTORY  0.0020833 //0.003125, this Factor doesnt saturate the system
#define FACTORTH  0.0055 //Rotation factor
#define FACTORZ  0.05 //Descend Factor

#define MAXV  0.5 //Max Vx and Vy speed
#define MINV -0.5 //Min Vx and Vy speed
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
    sub = po_nh.subscribe("/predicted_states", 10, &Controller::controllerCallBack, this); ////Predicted states subscriber (KF states)
    pub1 = po_nh.advertise<drone_controller::error>("/error",10) ;
    land_client = po_nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land"); //landing client
    lastTime = ros::Time::now(); //ROS time initialization throw ros time
    imageW = 640/2;  //Image W size
    imageH = 480/2;  //Image H size
    zini = 2.25; //Start altitude of search
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
	float ErZ = abs(width-heigh); //Error in W and H of the images

	//Variables to be published (Initialized in 0)
	float vx = 0;
	float vy = 0;
	float vthe = 0;
	float zpos = 0;


	//Info Printer of the errors
        ROS_INFO("Marker = (%f , %f) \n timeBetweenMarkers = %fs | Yaw error = %f deg | W and H error = %f px  \n", centX, centY, timeBetweenMarkers, ErTheta, ErZ);	

	//Calculate Z descend based on W and H
	if(ErZ<5) //If the error of W and H is less than 3 pixels
	{
	    zpos =  zini -  FACTORZ; //Descend Z based on the factor 
	}
	else{
	    zpos = zini; //If there is more than 3 pixels of error, hold pos
	}

	//Drone service for automatic langind when it reaches an specific altitude
	if(zpos <= 0.5){ //If the pos in z is less than this altitude
	mavros_msgs::CommandTOL land_cmd; //Set all the descend parameters to Zero
        land_cmd.request.yaw = 0;
        land_cmd.request.latitude = 0;
        land_cmd.request.longitude = 0;
        land_cmd.request.altitude = 0;

	//When it lands, everything goes to zero
        if (!(land_client.call(land_cmd) && land_cmd.response.success)){ //Publish the service of landing
        ROS_INFO("Landing");
	vx = 0; //Set all the velocities and positions to 0
        vy = 0;
	vthe = 0;
	zpos = 0;

	drone_controller::error data;
	data.errorX = ErX;
	data.errorY = ErY;
	data.errorT = ErTheta;
	data.errorS = ErZ;
	
	printf("Error at Vx, Vy, Theta and Z are (%f,%f,%f,%f) \n", ErX, ErY, ErTheta, ErZ);
	pub1.publish(data);

	ros::shutdown(); //Shutdowm the node
        }
	}

	zini = zpos; //Zini update to dont crash the drone
	

	// Calculate Vx. Vy and Yaw rate
        vx = -1 * ErX * FACTORX; //Ex for a multiplication factor
        vy = ErY * FACTORY; //Ey for a multiplication factor
	vthe = -1 * ErTheta * FACTORTH; //Etheta for a multiplication factor
	

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
        pos.position.z = zpos;
        pos.velocity.x = vx;//X(+) Drone left, X(-) Drone Right
        pos.velocity.y = vy;//Y(+) Drone Backward, X(-) Drone Forward
        pos.yaw_rate = vthe;//////Yaw(+) Drone Anti horario, Yaw(-) Drone Horario

	printf("Dany Vx, Vy, Vthe and Zpos values at (%f,%f,%f,%f) \n", vx, vy, vthe, zpos);
	pub.publish(pos);

	drone_controller::error data;
	data.errorX = ErX;
	data.errorY = ErY;
	data.errorT = ErTheta;
	data.errorS = ErZ;
	
	printf("Error at Vx, Vy, Theta and Z are (%f,%f,%f,%f) \n", ErX, ErY, ErTheta, ErZ);
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

