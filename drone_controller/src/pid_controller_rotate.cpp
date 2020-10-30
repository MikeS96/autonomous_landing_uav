/**
 *  @file pid_controller_rotate.cpp
 *  @author Miguel Saavedra (miguel.saaruiz@gmail@gmail.com)
 *  @brief PID controller to move the vehicle towards the center of the 
 *  landing platform and orientate it WRT x axis.
 *  @version 0.1
 *  @date 05-01-2020
 * 
 *  Copyright (c) 2020 Miguel Saavedra
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */


#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include "mavros_msgs/PositionTarget.h"
#include "object_detector/States.h" // Custom msgs of type States
#include "drone_controller/Error.h" // Custom msgs of type Error
#include "drone_controller/pid.h"

class Controller
{
    private: 
        //Private class atributes
        ros::NodeHandle po_nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher pub1;
        ros::Time lastTime;
        double imageW;
        double imageH;
        PID* pidx; // PID objects
        PID* pidy;
        PID* pidth;

    public:
        // Public class attributes and methods
        Controller(ros::NodeHandle ao_nh) : po_nh(ao_nh) 
        {
            // PID controllers objects
            pidx = new PID(0.75, -0.75, 0.00234375, 0.00046875, 0.0); // max, min, kp, kd, ki
            pidy = new PID(0.75, -0.75, 0.003125, 0.000625, 0.0);
            pidth = new PID(0.5, -0.5, 0.0055, 0.0011, 0.0);
            // Publisher type mavros_msgs::PositionTarget, it publishes in /mavros/setpoint_raw/local topic
            pub = po_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10) ; 
            // Publisher type drone_controller::Error, it publishes in /error topic
            pub1 = po_nh.advertise<drone_controller::Error>("/error",10) ;
            // Subscriber to /predicted_states topic from object_detector/Corners
            sub = po_nh.subscribe("/predicted_states", 10, &Controller::controllerCallBack, this); 
            lastTime = ros::Time::now(); // ROS time initialization throw ros time
            imageW = 640/2;  // Setpoint in X
            imageH = 480/2;  // Setpoint in Y
        }

        void controllerCallBack(const object_detector::States& msg) //Callback para el subscriber
        {
            // Time since last call
            // double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec(); //The average publish time is of 10ms
            // lastTime = ros::Time::now();

            // Error Calculation between image and template's center
            float ErX = imageW - msg.Xc; // Error in X of the image
            float ErY = imageH - msg.Yc; //Error in Y of the image
            float ErTheta = msg.Theta; // Error in Theta of the image

            // Publish the error
            drone_controller::Error er;
            er.errorX = ErX;
            er.errorY = ErY;
            er.errorT = ErTheta;
            er.errorS = 0;

            //Velocities
            float Vx = 0;
            float Vy = 0;
            float Vthe = 0;

            // Time since last call
            double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec();
            lastTime = ros::Time::now();

            // Compute controller output for each axis
            Vy = (float) pidx->calculate(imageW, msg.Xc, timeBetweenMarkers); // Setpoint, Process Variable, Sample time for Vx
            Vx = (float) pidy->calculate(imageH, msg.Yc, timeBetweenMarkers); 
            Vthe = (float) pidth->calculate(0, msg.Theta, timeBetweenMarkers);

            // Position target object to publish
            mavros_msgs::PositionTarget pos;

            //FRAME_LOCAL_NED to move WRT to body_ned frame
            pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

            pos.header.stamp = ros::Time::now(); // Time header stamp
            pos.header.frame_id = "base_link"; // "base_link" frame to compute odom
            pos.type_mask = 1987; // Mask for Vx, Vy, Z pos and Yaw rate
            pos.position.z = 2.0f;
            pos.velocity.x = Vx;
            pos.velocity.y = Vy;
            pos.yaw_rate = Vthe;

            printf("PID Vx and Vy values at (%f,%f) \n", Vx, Vy);
            pub.publish(pos);


            printf("Error at Vx, Vy and Theta are (%f,%f,%f) \n", ErX, ErY, ErTheta);
            pub1.publish(er);
        }
};

int main(int argc, char** argv)
{   

    ros::init(argc, argv, "controller_node"); //Nombre del nodo que uso para suscribirme y publicar la info
    ros::NodeHandle n;
    Controller cont(n);


    ros::spin();

    return 0;
}

