/**
 *  @file cent_calculator.cpp
 *  @author Miguel Saavedra (miguel.saaruiz@gmail@gmail.com)
 *  @brief This node computes the centroid, height, widhth and angle WRT to x
 *  of a template
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
#include <find_object_2d/ObjectsStamped.h>
#include "object_detector/Corners.h" // Custom msg of type Corners
#include "object_detector/States.h" // Custom msgs of type States

# define M_PI   3.14159265358979323846  /* pi */

class States
{

	private: 
		// Private class attributes
		ros::NodeHandle po_nh;
		ros::Subscriber sub;
		ros::Publisher pub;


	public:
		// Public class attributes and methods
		States(ros::NodeHandle ao_nh) : po_nh( ao_nh )
		{
			// Publisher type object_detector::States, it publishes in /states topic
			pub = po_nh.advertise<object_detector::States>( "/states", 10) ; 
			// Subscriber to /corners topic from object_detector/Corners
			sub = po_nh.subscribe("/corners", 10, &States::centCalculator, this); 
		}

		// Subscriber callback
		void centCalculator(const object_detector::Corners& msg) //Callback para el subscriber "CentCalculator"
		{	
			// Creation of a States object to publish the info
			object_detector::States st;

			// Theta angle calculation
			float theta = computeTheta(msg.BottomLeftX, msg.BottomLeftY,
										msg.BottomRightX, msg.BottomRightY);
			// Bottom vector magnitude
			float v_bot = computeEucDist(msg.BottomLeftX, msg.BottomLeftY,
										msg.BottomRightX, msg.BottomRightY);
			// Right vector magnitude
			float v_right = computeEucDist(msg.BottomRightX, msg.BottomRightY,
										msg.TopRightX, msg.TopRightY);
			// Top vector magnitude
			float v_top = computeEucDist(msg.TopLeftX, msg.TopLeftY,
										msg.TopRightX, msg.TopRightY);
			// Left vector magnitude
			float v_left = computeEucDist(msg.TopLeftX, msg.TopLeftY,
										msg.BottomLeftX, msg.BottomLeftY);

			// Centroid of the template in the image
			st.Xc = msg.CenterX;
			st.Yc = msg.CenterY;
			// Computation of width and height as an average of the bottom and top magnitudes
			st.W = (v_bot + v_top) / 2; 
			st.H = (v_right + v_left) / 2;
			st.Theta = theta;

			// Correction of theta when the angle is bigger than 90 degrees to avoid bad orientation for the controller
			if(theta > 90) 
			{
				int cont = theta / 90;
				st.Theta = theta - cont * 90;
			}
			else if(theta < 90)
			{
				int cont = -1 * theta / 90;
				st.Theta = theta + cont * 90;
			}

			// Uncomment these lines to print the results on console
			/*printf("Centroid in pix position (%f,%f)\n Object Width (%f)\n Object Height (%f)\n Theta (%f)\n", 
							
					st.Xc, st.Yc,
					st.W, st.H,
					st.Theta);
			*/
			pub.publish(st);
		}

		/**
		 * Calculate the euclidian distance between two points
		 *
		 *
		 * @param x_1 coordiante x of first point
		 * @param x_2 coordiante x of first point
		 * @param x_3 coordiante x of first point
		 * @param x_4 coordiante x of first point
		 * @return norm euclidian dist of points
		 */
		float computeEucDist(const float x_1, const float y_1, const float x_2, const float y_2) 
		{
			float vector_x = x_2 - x_1;
			float vector_y = y_2 - y_1;
			return sqrt(abs(vector_x * vector_x) + abs(vector_y * vector_y));
		}

		/**
		 * Calculate the angle WRT X
		 *
		 *
		 * @param x_1 coordiante x of first point
		 * @param x_2 coordiante x of first point
		 * @param x_3 coordiante x of first point
		 * @param x_4 coordiante x of first point
		 * @return th angle WRT X
		 */
		float computeTheta(const float x_1, const float y_1, const float x_2, const float y_2) 
		{
			float vector_x = x_2 - x_1;
			float vector_y = y_2 - y_1;
			return atan2(vector_y, vector_x) * 180 / M_PI;
		}
};


int main(int argc, char** argv)
{   
    ros::init(argc, argv, "data_calculation"); // Node name
    ros::NodeHandle n;
    States compt(n);
    ros::spin();

    return 0;
}
