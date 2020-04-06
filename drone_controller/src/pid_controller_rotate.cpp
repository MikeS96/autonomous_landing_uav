#include <ros/ros.h>


#include <stdio.h>
#include <math.h>

#include "mavros_msgs/PositionTarget.h"
#include "drone_controller/states.h"
#include "drone_controller/error.h"
#include "drone_controller/pid.h"


PID pidx = PID(0.75, -0.75, 0.00234375, 0.00046875, 0.0); //max, min, kp, kd, ki (Max = 1m/s, Min = 1m/s, 
PID pidy = PID(0.75, -0.75, 0.003125, 0.000625, 0.0);

PID pidyth = PID(0.5, -0.5, 0.0055, 0.0011, 0.0);
// PID pidy = PID(1, -1, 0.003125, 0.0005, 0.0); works good kd 0.000625
class Controller
{
  private: //Declaro las variables de mis publisher y subscribers

  ros::NodeHandle po_nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher pub1;
  ros::Time lastTime;
  double imageW;
  double imageH;


  public:
  Controller(ros::NodeHandle ao_nh) : po_nh(ao_nh) //Constructor de la clase Controller
  {
    pub = po_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10) ; //Topico endiablado que me deja controlar velocidades y posiciones
    pub1 = po_nh.advertise<drone_controller::error>("/error",10) ;
    sub = po_nh.subscribe("/predicted_states", 10, &Controller::controllerCallBack, this); //Suscriber del tipo find_object_2d/ObjectsStamped que se suscribe a /objects
    lastTime = ros::Time::now();
    imageW = 640/2;  //Image size
    imageH = 480/2;
  }

  void controllerCallBack(const drone_controller::states& msg) //Callback para el subscriber
  {

    //Image data
    double centX = (double) msg.Xc; //Parsing float into double
    double centY = (double) msg.Yc;
    double width = (double) msg.W;
    double heigh = (double) msg.H;
    double theta = (double) msg.Theta;

    //Velocities
    float Vx = 0;
    float Vy = 0;
    float Vthe = 0;

    // Time since last call
    double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec();
    lastTime = ros::Time::now();


    Vx = (float)  pidx.calculate(imageW, centX, timeBetweenMarkers); //Setpoint, Process Variable, Sample time for Vx
    Vy = (float) -1 * pidy.calculate(imageH, centY, timeBetweenMarkers); //Setpoint, Process Variable, Sample time for Vy
    Vthe = (float) pidyth.calculate(0, theta, timeBetweenMarkers); //Setpoint, Process Variable, Sample time for Vy

    //Pos object
    mavros_msgs::PositionTarget pos; //Creacion del objeto tipo setpoint_raw/local

    pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;  //FRAME_LOCAL_NED to move the drone frame

    pos.header.stamp = ros::Time::now(); //Time header stamp
    pos.header.frame_id = "base_link"; //"base_link" frame to compute odom
    pos.type_mask = 1987; //Mask for Vx, Vy, Z pos and Yaw rate
    pos.position.z = 2.0f;//0.001*some_object.position_z;
    pos.velocity.x = Vx;//X(+) Drone left, X(-) Drone Right
    pos.velocity.y = Vy;//Y(+) Drone Backward, X(-) Drone Forward
    pos.yaw_rate = Vthe;//0.001*some_object.position_y;

    printf("PID Vx and Vy values at (%f,%f) \n", Vx, Vy);
    pub.publish(pos);

    drone_controller::error data;
    data.errorX = imageW - centX;
    data.errorY = imageH - centY;
    data.errorT = theta;
    data.errorS = 0;

    printf("Error at Vx, Vy and Theta are (%f,%f,%f) \n", data.errorX, data.errorY, data.errorT);
    pub1.publish(data);
  }


};//End of class 


int main(int argc, char** argv)
{   

    ros::init(argc, argv, "controller_node"); //Nombre del nodo que uso para suscribirme y publicar la info
    ros::NodeHandle n;
    Controller cont(n);


    ros::spin();

    return 0;
}

