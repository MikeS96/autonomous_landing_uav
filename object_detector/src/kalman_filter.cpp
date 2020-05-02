/**
 *  @file kalman_filter.cpp
 *  @author Miguel Saavedra (miguel.saaruiz@gmail@gmail.com)
 *  @brief Linear kalman filter implementation for states estimation
 *  @version 0.1
 *  @date 04-30-2020
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
#include "object_detector/States.h" // Custom msg of type States
#include <Eigen/Dense>

# define M_PI 3.14159265358979323846  /* pi */

using namespace Eigen;

class Kalman
{
    private:
        // Private class attributes
        ros::NodeHandle po_nh;
        ros::Subscriber sub;
        ros::Publisher pub;

        MatrixXd T; // Posteriori estimate covariance matrix
        MatrixXd Q; // Covariance of the process noise
        MatrixXd R; // Covariance of the observation noise
        MatrixXd A; // State transition matrix
        MatrixXd H; // Observation model
        MatrixXd X; // State vector
        MatrixXd Z; // Innovation vectot
        MatrixXd S1; // Covariance of the innovation
        MatrixXd Kg; // Kalman gain
        float dt; // Delta of time
        int first_iter; // variable to check the first iteration of the algorithm

    public:
        // Public class attributes and methods
        Kalman(ros::NodeHandle ao_nh) : po_nh( ao_nh ), first_iter(0), dt(1), T(10,10), Q(10,10), R(5,5), 
                                        A(10,10), H(5,10), X(10,1), Z(5,1), S1(5,5), Kg(5,5)
        {
            // Publisher type object_detector::States, it publishes in /predicted_states topic
            pub = po_nh.advertise<object_detector::States>( "/predicted_states", 10 ) ;
            // Subscriber to /states topic from object_detector/States
            sub = po_nh.subscribe("/states", 10, &Kalman::predictionsDetectedCallback, this); 
            // Delta of time for the transition matrix
            this->dt = 0.1;
            this->first_iter = 0;


            // Posteriori estimate covariance matrix initialization
            this->T <<  2,0,0,0,0,0,0,0,0,0, 0,2,0,0,0,0,0,0,0,0, 
                        0,0,5,0,0,0,0,0,0,0, 0,0,0,5,0,0,0,0,0,0, 
                        0,0,0,0,5.625,0,0,0,0,0, 0,0,0,0,0,1e-3,0,0,0,0, 
                        0,0,0,0,0,0,1e-3,0,0,0, 0,0,0,0,0,0,0,1e-3,0,0, 
                        0,0,0,0,0,0,0,0,1e-3,0, 0,0,0,0,0,0,0,0,0,1e-3;

            /* Covariance matrix 
            * Xc  [2 0 0  0  0  0  0   0   0  0]
            * Yc  [0 2 0  0  0  0  0   0   0  0]
            * W   [0 0 5  0  0  0  0   0   0  0]
            * H   [0 0 0  5  0  0  0   0   0  0]
            * Th  [0 0 0  0  5.625  0  0   0   0  0]
            * Xc' [0 0 0  0  0  1e-3  0   0   0  0]
            * Yc' [0 0 0  0  0  0  1e-3   0   0  0]
            * W'  [0 0 0  0  0  0  0   1e-3   0  0]
            * H'  [0 0 0  0  0  0  0   0   1e-3  0]
            * Th' [0 0 0  0  0  0  0   0   0  1e-3] 
            */

            // Covariance of the process noise initialization
            this->Q = 1e-4*MatrixXd::Identity(10,10);
            // Covariance of the observation noise initialization
            this->R = 1e-2*MatrixXd::Identity(5,5);
            this->R(0, 0) = 1e-4;
            this->R(1, 1) = 1e-4;

            // State vector initialization
            this->X = MatrixXd::Zero(10,1); 
            // Innovation vectot initialization
            this->Z = MatrixXd::Zero(5,1); 
            // Covariance of the innovation initialization
            this->S1 = MatrixXd::Zero(5,5); 
            // Kalman gain initialization
            this->Kg = MatrixXd::Zero(5,5); 

            // State transition matrix initialization
            this->A << 1,0,0,0,0,dt,0,0,0,0, 0,1,0,0,0,0,dt,0,0,0, 
                        0,0,1,0,0,0,0,dt,0,0, 0,0,0,1,0,0,0,0,dt,0, 
                        0,0,0,0,1,0,0,0,0,dt, 0,0,0,0,0,1,0,0,0,0, 
                        0,0,0,0,0,0,1,0,0,0, 0,0,0,0,0,0,0,1,0,0, 
                        0,0,0,0,0,0,0,0,1,0, 0,0,0,0,0,0,0,0,0,1;
              
            /* Transition model 
            * Xc  [1 0 0  0  0  dt  0   0   0  0]
            * Yc  [0 1 0  0  0  0  dt   0   0  0]
            * W   [0 0 1  0  0  0  0   dt   0  0]
            * H   [0 0 0  1  0  0  0   0   dt  0]
            * Th  [0 0 0  0  1  0  0   0   0  dt]
            * Xc' [0 0 0  0  0  1  0   0   0  0]
            * Yc' [0 0 0  0  0  0  1   0   0  0]
            * W'  [0 0 0  0  0  0  0   1   0  0]
            * H'  [0 0 0  0  0  0  0   0   1  0]
            * Th' [0 0 0  0  0  0  0   0   0  1] 
            */

            // Observation model initialization
            this->H << 1,0,0,0,0,0,0,0,0,0, 
                        0,1,0,0,0,0,0,0,0,0, 
                        0,0,1,0,0,0,0,0,0,0, 
                        0,0,0,1,0,0,0,0,0,0, 
                        0,0,0,0,1,0,0,0,0,0;

            /* Transition model 
            * Xc  [1 0 0  0  0  0  0   0   0  0]
            * Yc  [0 1 0  0  0  0  0   0   0  0]
            * W   [0 0 1  0  0  0  0   0   0  0]
            * H   [0 0 0  1  0  0  0   0   0  0]
            * Th  [0 0 0  0  1  0  0   0   0  0]
            */
        }

        // Subscriber callback
        void predictionsDetectedCallback(const object_detector::States& msg)
        {	
            // Creation of a States object to publish the info
            object_detector::States predictions;

            MatrixXd measurement(5,1); // Vector to store the observations
            measurement = MatrixXd::Zero(5,1);

            // If it is the first iteration, initialize the states with the first observation
            if(this->first_iter==0)
            {
                this->X(0,0) = msg.Xc; // State xc
                this->X(1,0) = msg.Yc; // State Yc
                this->X(2,0) = msg.W; // State Width
                this->X(3,0) = msg.H; // State Height
                this->X(4,0) = msg.Theta; // State Theta
                this->X(5,0) = 0; // State Xc'
                this->X(6,0) = 0; // State Yc'
                this->X(7,0) = 0; // State Width'
                this->X(8,0) = 0; // State Height'
                this->X(9,0) = 0; // State Theta'
                this->first_iter = 1;
            }

            // Prediction step
            this->X = this->A*this->X; 
            this->T = ((this->A*this->T)*this->A.transpose())+this->Q;

            // Update step, assign the observations to the measurement vector 
            measurement(0,0) = msg.Xc;
            measurement(1,0) = msg.Yc;
            measurement(2,0) = msg.W;
            measurement(3,0) = msg.H;
            measurement(4,0) = msg.Theta;

            // If there is a valid measurement (the detector found the coordinates)
            if((measurement(0)>0)&&(measurement(1)>0)&&(measurement(2)>0)&&(measurement(3)>0)) 
            {
                // Update step
                this->Z = measurement - this->H*this->X; 
                this->S1 = ((this->H*this->T)*this->H.transpose())+R; 
                this->Kg = (this->T*this->H.transpose())*this->S1.inverse(); 
                this->X = this->X + this->Kg*this->Z; 
                this->T = (MatrixXd::Identity(10,10)-(this->Kg*this->H))*this->T; 
            }

            // Assign the predictions to the publisher object
            predictions.Xc = this->X(0,0);
            predictions.Yc = this->X(1,0);
            predictions.W = this->X(2,0);
            predictions.H = this->X(3,0);
            predictions.Theta = this->X(4,0);

            // Uncomment these lines to print the results on console
            /* printf("The Centroid predicted with KF are (%f,%f)\n The Width is (%f)\n The Height is (%f)\n Theta is (%f)\n", 

            	predictions.Xc, predictions.Yc,
            	predictions.W, predictions.H,
            	predictions.Theta);
            */
            
            pub.publish(predictions);
        }
};


int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "KF_predictor"); 
    ros::NodeHandle n;
    Kalman kf(n);
    ros::spin();

    return 0;
}   
