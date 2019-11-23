#include <ros/ros.h>
#include <find_object_2d/ObjectsStamped.h> //find_object_2d include
#include <std_msgs/Float32MultiArray.h> 
#include "object_detector/corners.h" //Custom message where im going to publish 
#include <QTransform> //Library to do the perspective transforms

class Detections //Class Detections
{
  private: //Declaration of pubs and subs

  ros::NodeHandle po_nh;
  ros::Subscriber sub;
  ros::Publisher pub;

  public:
  Detections(ros::NodeHandle ao_nh) : //Constructor of the class detections
    po_nh( ao_nh )
  {
    pub = po_nh.advertise<object_detector::corners>( "/corners", 10 ) ; //Publisher del tipo object_detector/corners que publica al topic corners_cal
    sub = po_nh.subscribe("/objects", 10, &Detections::cornersDetectedCallback, this); //Suscriber del tipo find_object_2d/ObjectsStamped que se suscribe a /objects
  }

  void cornersDetectedCallback(const std_msgs::Float32MultiArrayConstPtr& msg) //Callback para el subscriber
  {
        object_detector::corners info; //Creationg of object kind corners for info managment

	const std::vector<float> & data = msg->data; //data saving in temporal variables
	if(data.size())
	{
		for(unsigned int i=0; i<data.size(); i+=12) //For to extract data given by the find object 2d topic
		{
			// get data
			int id = (int)data[i];
			float objectWidth = data[i+1];
			float objectHeight = data[i+2];

			// Find corners Qt //Inicializo Constructor de la clase QTrasform y almacenamiento de la matriz de homografia
			QTransform qtHomography(data[i+3], data[i+4], data[i+5],
									data[i+6], data[i+7], data[i+8],
									data[i+9], data[i+10], data[i+11]);

                        //Creates and returns a QPointF object that is a copy of the given point, p, mapped into the coordinate system defined by this matrix.
                        //QPointF es una clase que permite dar la coordenada x y y de un punto con precisión Float
                        //.map Maps the given coordinates x and y into the coordinate system defined by this matrix. The resulting values are put in *tx and *ty, respectively.
                        //Esto mapea el objeto en el tamaño total de la imagen, dando las coordenadas de las ezquinas en la imagen de escena
			QPointF qtTopLeft = qtHomography.map(QPointF(0,0)); //top left coordinates
			QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0)); //top right coordinates
			QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight)); //bottom left coordinates
			QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight)); //bottom right coordinates
			QPointF qCenter = qtHomography.map(QPointF(objectWidth/2,objectHeight/2)); //Cent Coordinates
			

			float orden[4][2] = { {qtTopLeft.x(), qtTopLeft.y()}, {qtTopRight.x(), qtTopRight.y()}, {qtBottomLeft.x(), qtBottomLeft.y()}, {qtBottomRight.x(), qtBottomRight.y()} };// First Column is X, second Y. This Matrix save the pixel points
			float lan[1][2] = {0,0}; //Temporal variable to save the last position 

			for (int i = 0; i < 4; i++) //Buble sort to know the position of each corner
			{
				for (int j = 1; j < 4; j++) 
				{
	   				if (orden[j-1][0] > orden[j][0]) 
	   				{ 
	    				  lan[0][0] = orden[j][0];
					  lan[0][1] = orden[j][1];
					  orden[j][0] = orden[j-1][0];
					  orden[j][1] = orden[j-1][1];
	    				  orden[j-1][0] = lan[0][0];
					  orden[j-1][1] = lan[0][1];
	  				 }
				}
			}

			if(orden[0][1]>orden[1][1]) //Data publishing based on the buble sort 
			{
				info.BottomLeftX = orden[0][0];
				info.BottomLeftY = orden[0][1];
				info.TopLeftX = orden[1][0];
				info.TopLeftY = orden[1][1];
			}
			else 
			{
				info.BottomLeftX = orden[1][0];
				info.BottomLeftY = orden[1][1];
				info.TopLeftX = orden[0][0];
				info.TopLeftY = orden[0][1];
			}

			if(orden[2][1]>orden[3][1])
			{
				info.BottomRightX = orden[2][0];
				info.BottomRightY = orden[2][1];
				info.TopRightX = orden[3][0];
				info.TopRightY = orden[3][1];
			}
			else 
			{
				info.BottomRightX = orden[3][0];
				info.BottomRightY = orden[3][1];
				info.TopRightX = orden[2][0];
				info.TopRightY = orden[2][1];
			}

			info.CenterX = qCenter.x();
			info.CenterY = qCenter.y();
			//Uncomment these lines to print the results on console
			/*printf("Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n Qt Center at (%f,%f)\n", //Impresion en consola de los datos de los corners
				
					info.TopLeftX, info.TopLeftY,
					info.TopRightX, info.TopRightY,
					info.BottomLeftX, info.BottomLeftY,
					info.BottomRightX, info.BottomRightY, info.CenterX, info.CenterY);
			*/
			pub.publish(info);
		}
	}
	else    //This else publish zeros in case there is no detection by the find object 2d
	{	//Uncomment this line to print the results on console
		//printf("No objects detected.\n");
		info.TopLeftX = 0;
		info.TopLeftY = 0;
		info.TopRightX = 0;
		info.TopRightY = 0;
		info.BottomLeftX = 0;
		info.BottomLeftY = 0;
		info.BottomRightX = 0;
		info.BottomRightY = 0;
		info.CenterX = 0;
		info.CenterY = 0;
		pub.publish(info);
	}
        
  }


};//End of class 


int main(int argc, char** argv)
{   

    ros::init(argc, argv, "corners_detected"); //Node name "Corners_detected"
    ros::NodeHandle n;
    Detections detec(n);

    ros::spin();

    return 0;
}
