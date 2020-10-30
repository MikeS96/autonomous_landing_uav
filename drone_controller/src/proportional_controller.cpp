/**
 *  @file proportional_controller.cpp
 *  @author Miguel Saavedra (miguel.saaruiz@gmail@gmail.com)
 *  @brief Proportional controller to land the vehicle on the landing pad
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
#include "mavros_msgs/PositionTarget.h" // Mavros topic to control vel and pos
#include "object_detector/States.h" // Custom msgs of type States
#include "drone_controller/Error.h" // Custom msgs of type Error
#include <mavros_msgs/CommandTOL.h> // Service for landing


#define FACTOR  0.003125 // Vx and Vy proportional gain
#define FACTORTH  0.0055 // Theta proportional gain
#define FACTORZ  0.05 // Descend Factor
#define MAXV  1 // Max Vx and Vy speed
#define MINV -1 // Min Vx and Vy speed
#define MAXR  0.5 // Max Yaw rate
#define MINR -0.5 // Min Yaw rate


class Controller 
{
    private: 
        //Private class atributes
        ros::NodeHandle po_nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher pub1;
        ros::Time lastTime;
        ros::ServiceClient land_client;

        float imageW; // Image Width
        float imageH; // Image Height
        float zini; // Initial height pos

    public: 
        // Public class attributes and methods
        Controller(ros::NodeHandle ao_nh) : po_nh(ao_nh) 
        {
            // Publisher type mavros_msgs::PositionTarget, it publishes in /mavros/setpoint_raw/local topic
            pub = po_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10) ; 
            // Publisher type drone_controller::Error, it publishes in /error topic
            pub1 = po_nh.advertise<drone_controller::Error>("/error",10) ;
            // Subscriber to /predicted_states topic from object_detector/Corners
            sub = po_nh.subscribe("/predicted_states", 10, &Controller::controllerCallBack, this); 
            // Landing client
            land_client = po_nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
            lastTime = ros::Time::now(); // ROS time initialization throw ros time
            imageW = 640/2;  // Setpoint in X
            imageH = 480/2;  // Setpoint in Y
            zini = 4.0; // Initial alttitude
        }

        // Subscriber callback
        void controllerCallBack(const object_detector::States& msg) 
        {
            // Time since last call
            // double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec(); //The average publish time is of 10ms
            // lastTime = ros::Time::now();

            // Error Calculation between image and template's center
            float ErX = imageW - msg.Xc; // Error in X of the image
            float ErY = imageH - msg.Yc; //Error in Y of the image
            float ErTheta = msg.Theta; // Error in Theta of the image
            float ErZ = abs(msg.W - msg.H); //Error in W and H of the images

            // Publish the error
            drone_controller::Error er;
            er.errorX = ErX;
            er.errorY = ErY;
            er.errorT = ErTheta;
            er.errorS = ErZ;

            // Variables to be published (Initialized in 0)
            float vx = 0;
            float vy = 0;
            float vthe = 0;
            float zpos = 0;	

            // If the erroe between width and height is less than 3 pixels
            if(ErZ < 3)
            {
                zpos =  zini -  FACTORZ; // Descend Z based on the factor 
            }
            else{
                zpos = zini; // If there is more than 3 pixels of error, hold pos
            }

            // Drone service for automatic langind when it reaches an specific altitude
            if(zpos <= 0.9)
            { 
                mavros_msgs::CommandTOL land_cmd; // Set all the descend parameters to Zero
                land_cmd.request.yaw = 0;
                land_cmd.request.latitude = 0;
                land_cmd.request.longitude = 0;
                land_cmd.request.altitude = 0;

                // When it lands, everything goes to zero
                if (!(land_client.call(land_cmd) && land_cmd.response.success))
                {   
                    // Publish the service of landing
                    ROS_INFO("Landing");                  
                    // Print final Error
                    printf("Error at Vx, Vy, Theta and Z are (%f,%f,%f,%f) \n", ErX, ErY, ErTheta, ErZ);
                    pub1.publish(er);
                    ros::shutdown(); // Shutdowm the node
                }
            }

            // Update vehicle's position
            zini = zpos; 

            // Calculate proportional gain for each axis
            vy = ErX * FACTOR; 
            vx = ErY * FACTOR; 
            vthe = ErTheta * FACTORTH;

            // Limit output 
            max_output(MAXV, MINV, vx); // Limit max Vx output
            max_output(MAXV, MINV, vy); // Limit max Vy output
            max_output(MAXR, MINR, vthe); // Limit max Vtheta output

            // Position target object to publish
            mavros_msgs::PositionTarget pos;

            //FRAME_LOCAL_NED to move WRT to body_ned frame
            pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;  

            pos.header.stamp = ros::Time::now(); // Time header stamp
            pos.header.frame_id = "base_link"; // "base_link" frame to compute odom
            pos.type_mask = 1987; // Mask for Vx, Vy, Z pos and Yaw rate
            pos.position.z = zpos;
            pos.velocity.x = vx;
            pos.velocity.y = vy;
            pos.yaw_rate = vthe;

            printf("Proportional Vx, Vy, Vthe and Zpos values at (%f,%f,%f,%f) \n", vx, vy, vthe, zpos);
            pub.publish(pos);

            printf("Error at Vx, Vy, Theta and Z are (%f,%f,%f,%f) \n", ErX, ErY, ErTheta, ErZ);
            pub1.publish(er);
        }

        /**
         * Limit the max output of the controller
         *
         *
         * @param max_v max output value
         * @param min_v min output value
         * @param out controller output
         */
        void max_output(const float& max_v, const float& min_v, float& out) 
        {
            if (out > max_v)
            {
                out = max_v;
            } 
            else if (out < min_v)
            {
                out = min_v;
            }
        }
};


int main(int argc, char** argv)
{   

    ros::init(argc, argv, "p_controller"); 
    ros::NodeHandle n;
    Controller cont(n);
    ros::spin();

    return 0;
}


