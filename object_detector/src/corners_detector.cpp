/**
 *  @file corners_detector.cpp
 *  @author Miguel Saavedra (miguel.saaruiz@gmail@gmail.com)
 *  @brief Algorithm to detect the corners of a template in an image
 *  @version 0.1
 *  @date 04-29-2020
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
#include <vector>
#include <find_object_2d/ObjectsStamped.h> // find_object_2d objectStamped msj
#include <std_msgs/Float32MultiArray.h> 
#include "object_detector/Corners.h" // Custom msj with the corners and centroid of the object
#include <QTransform> // Library to do the perspective transforms

// Struct to sort the list of points with respect to X
struct order {
    bool operator() (QPointF& pt1, QPointF& pt2) { return (pt1.x() < pt2.x());}
} sorter;

class Detections
{

	private: 
	  	// Private class attributes
		ros::NodeHandle po_nh;
		ros::Subscriber sub;
		ros::Publisher pub;

	public:
		// Public class attributes and methods
		Detections(ros::NodeHandle ao_nh) : po_nh( ao_nh )
		{
			// Publisher type object_detector::corners, it publishes in /corners topic
			pub = po_nh.advertise<object_detector::Corners>( "/corners", 10 ) ; 
			// Subscriber to /objects topic from find_object_2d/ObjectsStamped
			sub = po_nh.subscribe("/objects", 10, &Detections::cornersDetectedCallback, this); 
		}

		// Subscriber callback
		void cornersDetectedCallback(const std_msgs::Float32MultiArrayConstPtr& msg) 
		{
			// Creation of a Corners object to publish the info
			object_detector::Corners coordinates; 
			// data saving in temporal variables
			const std::vector<float> & data = msg->data; 
			if(data.size())
			{
				for(unsigned int i=0; i<data.size(); i+=12) // For to extract data given by the find object 2d topic
				{
					// get data
					int id = (int)data[i];
					float objectWidth = data[i+1];
					float objectHeight = data[i+2];

					// Find corners Qt 
					// QTrasform initialization with the homography matrix
					QTransform qtHomography(data[i+3], data[i+4], data[i+5],
						data[i+6], data[i+7], data[i+8],
						data[i+9], data[i+10], data[i+11]);

			        // Map the template coordinates to the current frame with the shape of the template and homography
					QPointF qtTopLeft = qtHomography.map(QPointF(0,0)); // Top left coordinates
					QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0)); // Top right coordinates
					QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight)); // Bottom left coordinates
					QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight)); // Bottom right coordinates
					QPointF qCenter = qtHomography.map(QPointF(objectWidth/2,objectHeight/2)); // Centroid Coordinates

					// First Column is X coordinate, second is Y.
					std::vector<QPointF> points {qtTopLeft, qtTopRight, qtBottomLeft, qtBottomRight};

					// Sort the points with respect to X inside the vector of points
					std::sort(points.begin(), points.end(), sorter);

					// Assign coordinates to the publishing object
					assignCorners(points, coordinates);

					// Assign centroid values
					coordinates.CenterX = qCenter.x();
					coordinates.CenterY = qCenter.y();

					// Uncomment these lines to print the results on console
					/*printf("Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n Qt Center at (%f,%f)\n",
							coordinates.TopLeftX, coordinates.TopLeftY,
							coordinates.TopRightX, coordinates.TopRightY,
							coordinates.BottomLeftX, coordinates.BottomLeftY,
							coordinates.BottomRightX, coordinates.BottomRightY, 
							coordinates.CenterX, coordinates.CenterY);
					*/
				}
			}

			// Assign zero to all the members of the publishing object
			else   
			{	
				coordinates.TopLeftX = 0, coordinates.TopLeftY = 0;
				coordinates.TopRightX = 0, coordinates.TopRightY = 0;
				coordinates.BottomLeftX = 0, coordinates.BottomLeftY = 0;
				coordinates.BottomRightX = 0, coordinates.BottomRightY = 0;
				coordinates.CenterX = 0, coordinates.CenterY = 0;
				
			}
			// Publish the coordinates
			pub.publish(coordinates);
		}

		/**
		 * Assign the corners of the publishing object
		 *
		 * Assign the proper corner to each element of the 
		 * corners publishing object
		 *
		 * @param p vector of points.
		 * @param c publishing object with the sorted corners
		 */
		void assignCorners(const std::vector<QPointF>& p, object_detector::Corners& c) 
		{
			// Assign the bottom left and top left corners
			if(p[0].y() > p[1].y()) 
				{
					c.BottomLeftX = p[0].x();
					c.BottomLeftY = p[0].y();
					c.TopLeftX = p[1].x();
					c.TopLeftY = p[1].y();
				}
			else 
				{
					c.BottomLeftX = p[1].x();
					c.BottomLeftY = p[1].y();
					c.TopLeftX = p[0].x();
					c.TopLeftY = p[0].y();
				}

			// Assign the bottom right and top right corners
			if(p[2].y() > p[3].y())
				{
					c.BottomRightX = p[2].x();
					c.BottomRightY = p[2].y();
					c.TopRightX = p[3].x();
					c.TopRightY = p[3].y();
				}
			else 
				{
					c.BottomRightX = p[3].x();
					c.BottomRightY = p[3].y();
					c.TopRightX = p[2].x();
					c.TopRightY = p[2].y();
				}
		}
};


int main(int argc, char** argv)
{   

    ros::init(argc, argv, "corners_detected"); //Node name "Corners_detected"
    ros::NodeHandle n;
    Detections detec(n);

    ros::spin();

    return 0;
}
