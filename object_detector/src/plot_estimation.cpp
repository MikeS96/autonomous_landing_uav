/**
 *  @file plot_estimation.cpp
 *  @author Miguel Saavedra (miguel.saaruiz@gmail@gmail.com)
 *  @brief Plot in an image the estimation of the template
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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "object_detector/States.h" // Custom msg of type States
#include <find_object_2d/ObjectsStamped.h>
#include <std_msgs/Float32MultiArray.h>

static const std::string OPENCV_WINDOW = "Estimation";

class Drawer 
{
    private:
        ros::NodeHandle nh_; 
        image_transport::ImageTransport it_; 
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_; 
        ros::Subscriber sub;
        float data[5];
  
    public: 
        Drawer() : it_(nh_), data()
        {
            // Subscriber to /image_raw topic
            image_sub_ = it_.subscribe("/quad_f450_camera/camera_link/raw_image", 10, 
                                        &Drawer::imageCb, this);
            // Subscriber to /predicted_states topic from object_detector/States
            sub = nh_.subscribe("/predicted_states", 10, &Drawer::estimationsDetectedCallback, this); 
            // Publisher type sensor_msgs, it publishes in /rectangle_draw/raw_image topic
            image_pub_ = it_.advertise("/rectangle_draw/raw_image", 10); 
            // Window name
            cv::namedWindow(OPENCV_WINDOW);
        }

        // Class destructor
        ~Drawer()
        {
            cv::destroyWindow(OPENCV_WINDOW); //destruyo el objeto namedWindow
        }

        // Subscriber callback to assign the values of the states to an array
        void estimationsDetectedCallback(const object_detector::States& msg) 
        {    
            this->data[0] = msg.Xc;
            this->data[1] = msg.Yc;
            this->data[2] = msg.W;
            this->data[3] = msg.H;
            this->data[4] = msg.Theta;
        }

        // Subscriber callback to assign the values of the states to an array
        void imageCb(const sensor_msgs::ImageConstPtr& msg) 
        {
            // CV_vridge image pointer
            cv_bridge::CvImagePtr cv_ptr; 
            namespace enc = sensor_msgs::image_encodings;

            try
            {
                // Copy image to cv_bridge pointer
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // Draw the object detected in the image frame
            rectangle_draw(cv_ptr->image);
            image_pub_.publish(cv_ptr->toImageMsg());
        }

        void rectangle_draw(cv::Mat img)
        {
            // Assign the observations to measurements vector     
            cv::Mat measurement = cv::Mat(5, 1, CV_32FC1, &this->data);

            // Assign the centroid of the platform to a point object
            cv::Point pt =  cv::Point(measurement.at<float>(0), measurement.at<float>(1));

            // Create a rotated rectangle object
            cv::RotatedRect rRect = cv::RotatedRect(pt, cv::Size2f(measurement.at<float>(2),
                                    measurement.at<float>(3)), measurement.at<float>(4));

            // Points with the corner coordinates of the object
            cv::Point2f vertices[4];

            // Take the four corners from the rRect object and assing those to vertices
            rRect.points(vertices);

            // Draw the lines of the rectangle
            for (unsigned int i = 0; i < 4; i++)
            {
                cv::line(img, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
            }

            // Draw a point in each vertices
            cv::circle(img,vertices[0], 5, cv::Scalar(255,0,255), cv::FILLED, 8,0); // Pink
            cv::circle(img,vertices[1], 5, cv::Scalar(0,0,255), cv::FILLED, 8,0);  // Red
            cv::circle(img,vertices[2], 5, cv::Scalar(255,0,0), cv::FILLED, 8,0); // Blue
            cv::circle(img,vertices[3], 5, cv::Scalar(0,255,0), cv::FILLED, 8,0); // Green

            // Show the image
            cv::imshow(OPENCV_WINDOW, img);
            cv::waitKey(3); 
        }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Rectangle_Detector");
  Drawer ic;
  ros::spin();
  return 0;
}
