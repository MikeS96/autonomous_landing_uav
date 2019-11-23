#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "object_detector/states.h" //Custom msg to recibe the info

#include <find_object_2d/ObjectsStamped.h>
#include <std_msgs/Float32MultiArray.h>

//static const std::string OPENCV_WINDOW = "New Rectangle Detected Window";

class Edge_Detector //Creo la clase Edge detector
{
  ros::NodeHandle nh_; //Creo objeto nh_ para ROS.
  image_transport::ImageTransport it_; //Estos son los atributos de la clase
  image_transport::Subscriber image_sub_; //Publisher de ROS
  image_transport::Publisher image_pub_; //Suscriber de ROS

  ros::Subscriber sub;

  float data[5];
  
public: 
  Edge_Detector() //Constructor de la clase, 
    : it_(nh_), data() //Inicializo el atributo it_ con nh_ de ROS
  { //ECAMBIAR ESTO "/iris/camera_red_iris/image_raw"
    // Subscribe to input video feed and publish output video feed /usb_cam/image_raw
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, 
      &Edge_Detector::imageCb, this); //Suscribo el parametro image_sub a el topico /usb_cam/image_raw, pasandole la referencia de la clase Edge_detector y estara revisando el metodo imageCB

    image_pub_ = it_.advertise("/rectangle_draw/raw_image", 10); //Publico el parametro image_pub a el topico edge_detector/raw_image, pasandole la referencia de la clase Edge_detector

    //cv::namedWindow(OPENCV_WINDOW); //Inicio el objeto namedWindow con el nombre Raw Image window

    sub = nh_.subscribe("/predicted_states", 10, &Edge_Detector::estimationsDetectedCallback, this); //Me subscribo a los estados predecidos por el filtro de Kalman

  }

  ~Edge_Detector() //Destructor de la clase edte_detector
  {
    //cv::destroyWindow(OPENCV_WINDOW); //destruyo el objeto namedWindow
  }

  void estimationsDetectedCallback(const object_detector::states& msg) //Callback para el subscriber
  {    
       this->data[0] = msg.Xc;
       this->data[1] = msg.Yc;
       this->data[2] = msg.W;
       this->data[3] = msg.H;
       this->data[4] = msg.Theta;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) //Metodo imageCB
  {
    cv_bridge::CvImagePtr cv_ptr; //Creo un objeto del tipo CvImagePyr perteneciente a el paquete cv_bridge
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 1 && cv_ptr->image.cols > 1){
	
	rectangle_draw(cv_ptr->image);
    	image_pub_.publish(cv_ptr->toImageMsg());

	}
  }

  void rectangle_draw(cv::Mat img)
  {     

	
	cv::Mat_<float> measurement(5,1);

	measurement(0) = this->data[0];
	measurement(1) = this->data[1];
	measurement(2) = this->data[2];
	measurement(3) = this->data[3];
	measurement(4) = this->data[4];

	//Rectangle centroid
	cv::Point pt =  cv::Point(measurement(0), measurement(1));

	//Generacion del rectangulo rotado
	cv::RotatedRect rRect = cv::RotatedRect(pt, cv::Size2f(measurement(2),measurement(3)), measurement(4));

	//Vector que contendra los vertices del rectangulo para dibujarlo
	cv::Point2f vertices[4];

	//Retorna los vertices del rectangulo
	rRect.points(vertices);


	for (int i = 0; i < 4; i++)
	{
	cv::line(img, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
	}

	cv::circle(img,vertices[0],5,cv::Scalar(255,0,255),CV_FILLED, 8,0); //rosado
	cv::circle(img,vertices[1],5,cv::Scalar(0,0,255),CV_FILLED, 8,0);  //Rojo
	cv::circle(img,vertices[2],5,cv::Scalar(255,0,0),CV_FILLED, 8,0); //Azul
	cv::circle(img,vertices[3],5,cv::Scalar(0,255,0),CV_FILLED, 8,0); //Verde

    	//cv::imshow(OPENCV_WINDOW, img);
    	//cv::waitKey(3);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Rectangle_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}
