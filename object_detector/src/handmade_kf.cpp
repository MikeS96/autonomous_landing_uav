#include <ros/ros.h>
#include "object_detector/states.h" //Custom msg to recibe the info
#include <Eigen/Dense>

# define M_PI           3.14159265358979323846  /* pi */

using namespace Eigen;

class Kalman
{
  private: //Declaro las variables de mis publisher y subscribers

  ros::NodeHandle po_nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  float dt;
  float flag;

  MatrixXd eo; //Matriz de covarianza de estados iniciales
  MatrixXd Q; //Matriz de covarianza error de prediccion
  MatrixXd R; // Matriz de covarianza de la medida (R)
  MatrixXd A; // Matriz de transicion de predichos
  MatrixXd H; // Matriz de transicion de observados
  MatrixXd X; //Matriz estados 
  MatrixXd Z; // Matriz de observacion
  MatrixXd S1; // error innovacion
  MatrixXd Kg; // Kalman gain
  


  public:
  Kalman(ros::NodeHandle ao_nh) : //Constructor de la clase Kalman
    po_nh( ao_nh ), flag(0), dt(1), eo(10,10), Q(10,10), R(5,5), A(10,10), H(5,10), X(10,1), Z(5,1), S1(5,5), Kg(5,5)
  {
    pub = po_nh.advertise<object_detector::states>( "/predicted_states", 10 ) ; //Publisher del tipo object_detector::KF_States que publica al topic /predicted_states
    sub = po_nh.subscribe("/states", 10, &Kalman::predictionsDetectedCallback, this); //Suscriber del tipo object_detector/states que se suscribe a /states
    this->dt = 0.1;
    this->flag = 0;


    //Inicializacion matrices de covarianza
    this->eo <<  2,0,0,0,0,0,0,0,0,0, 0,2,0,0,0,0,0,0,0,0, 0,0,5,0,0,0,0,0,0,0, 0,0,0,5,0,0,0,0,0,0, 0,0,0,0,5.625,0,0,0,0,0, 0,0,0,0,0,1e-3,0,0,0,0, 0,0,0,0,0,0,1e-3,0,0,0, 0,0,0,0,0,0,0,1e-3,0,0, 0,0,0,0,0,0,0,0,1e-3,0, 0,0,0,0,0,0,0,0,0,1e-3;

    this->Q = 1e-4*MatrixXd::Identity(10,10);
    this->R = 1e-2*MatrixXd::Identity(5,5);

    this->R(0, 0) = 1e-4;
    this->R(1, 1) = 1e-4;

    this->X = MatrixXd::Zero(10,1); //Lleno de zeros la matriz de estados inicialmente
    this->Z = MatrixXd::Zero(5,1); //Lleno de zeros la matriz de observacion
    this->S1 = MatrixXd::Zero(5,5); //Lleno de zeros la matriz de error de innovacion
    this->Kg = MatrixXd::Zero(5,5); //Lleno de zeros la matriz de la ganancia de kalman

    //Matriz de Transición A
    this->A << 1,0,0,0,0,dt,0,0,0,0, 0,1,0,0,0,0,dt,0,0,0, 0,0,1,0,0,0,0,dt,0,0, 0,0,0,1,0,0,0,0,dt,0, 0,0,0,0,1,0,0,0,0,dt, 0,0,0,0,0,1,0,0,0,0, 0,0,0,0,0,0,1,0,0,0, 0,0,0,0,0,0,0,1,0,0, 0,0,0,0,0,0,0,0,1,0, 0,0,0,0,0,0,0,0,0,1;
      
    	  /* Transition MODEL */
  // Xc  [1 0 0  0  0  dt  0   0   0  0]
  // Yc  [0 1 0  0  0  0   dt  0   0  0]
  // W   [0 0 1  0  0  0   0   dt  0  0]
  // H   [0 0 0  1  0  0   0   0   dt 0]
  // Th  [0 0 0  0  1  0   0   0   0 dt]
  // Xc' [0 0 0  0  0  1   0   0   0  0]
  // Yc' [0 0 0  0  0  0   1   0   0  0]
  // W'  [0 0 0  0  0  0   0   1   0  0]
  // H'  [0 0 0  0  0  0   0   0   1  0]
  // Th' [0 0 0  0  0  0   0   0   0  1]
 
    //Matriz de medición H 
    this->H << 1,0,0,0,0,0,0,0,0,0, 0,1,0,0,0,0,0,0,0,0, 0,0,1,0,0,0,0,0,0,0, 0,0,0,1,0,0,0,0,0,0, 0,0,0,0,1,0,0,0,0,0;
    

          /* Measurement MODEL */
  // Xc  [1 0 0  0  0  0   0   0   0  0]
  // Yc  [0 1 0  0  0  0   0   0   0  0]
  // W   [0 0 1  0  0  0   0   0   0  0]
  // H   [0 0 0  1  0  0   0   0   0  0]
  // Th  [0 0 0  0  1  0   0   0   0  0]

  }
  
  void predictionsDetectedCallback(const object_detector::states& msg) //Callback para el subscriber
  {	
    object_detector::states predictions; //object of  object_detector::KF_States type

    MatrixXd measurement(5,1); //Measurement matrix
    measurement = MatrixXd::Zero(5,1); //Parsing the measurement matrix into double
          

    //Esto garantiza a inicializacion del filtro de Kalman con la primera medicion
    if(this->flag==0){

      //Condicion inicial de los estados
      this->X(0,0) = msg.Xc; //State xc
      this->X(1,0) = msg.Yc; //State Yc
      this->X(2,0) = msg.W; //State Width
      this->X(3,0) = msg.H; //State Height
      this->X(4,0) = msg.Theta; //State Theta
      this->X(5,0) = 0; //State Xc'
      this->X(6,0) = 0; //State Yc'
      this->X(7,0) = 0; //State Width'
      this->X(8,0) = 0; //State Height'
      this->X(9,0) = 0; //State Theta'

      this->flag = 1;

	   }
	

  	// First predict, to update the internal statePre variable

  	this->X = this->A*this->X;  //Prediccion 
  	this->eo = ((this->A*this->eo)*this->A.transpose())+this->Q; //Transformacion lineal del error, para actualizar matriz covarianza e

  	//In case that there's not correct measurements, the output will be the prediction
  	predictions.Xc = this->X(0,0);
  	predictions.Yc = this->X(1,0);
  	predictions.W = this->X(2,0);
  	predictions.H = this->X(3,0);
  	predictions.Theta = this->X(4,0);
  	
  	// The update phase 
  	measurement(0,0) = msg.Xc;
  	measurement(1,0) = msg.Yc;
  	measurement(2,0) = msg.W;
  	measurement(3,0) = msg.H;
  	measurement(4,0) = msg.Theta;

  	//Condition to avoid correction if there's not measurement
  	if((measurement(0)>0)&&(measurement(1)>0)&&(measurement(2)>0)&&(measurement(3)>0)) {

    	this->Z = measurement - this->H*this->X; //Innovacion (Tengo mi duda si es asi la ecuacion)
    	this->S1 = ((this->H*this->eo)*this->H.transpose())+R; //Error de la innovacion
    	this->Kg = (this->eo*this->H.transpose())*this->S1.inverse(); //Calculo de la ganancia de Kalman
    	this->X = this->X + this->Kg*this->Z; //Caculo de X actualizado
    	this->eo = (MatrixXd::Identity(10,10)-(this->Kg*this->H))*this->eo; //Transformacion lineal del error, para actualizar matriz covarianza e
    	
    	//Updating the message with the upgrade phase
    	predictions.Xc = this->X(0,0); 
    	predictions.Yc = this->X(1,0);
    	predictions.W = this->X(2,0);
    	predictions.H = this->X(3,0);
    	predictions.Theta = this->X(4,0);

  	}
	
  	//Uncomment these lines to print the results on console
  	/*printf("The Centroid predicted with KF are (%f,%f)\n The Width is (%f)\n The Height is (%f)\n Theta is (%f)\n", //Impresion en consola de los datos de los corners
  				
  					predictions.Xc, predictions.Yc,
  					predictions.W, predictions.H,
  					predictions.Theta);
  	*/
  	pub.publish(predictions);
	
  }


};//End of class Kalman


int main(int argc, char** argv)
{   

    ros::init(argc, argv, "KF_predictor"); //Nombre del nodo que uso para suscribirme y publicar la info
    ros::NodeHandle n;
    Kalman detec(n);


    ros::spin();

    return 0;
}
