#include <ros/ros.h>
#include <find_object_2d/ObjectsStamped.h>
#include "object_detector/corners.h" //Custom msgs corners to recibe the corners from the corners detector node
#include "object_detector/states.h" //Custom msgs states t publish the states to KF
#include <QTransform>

# define M_PI   3.14159265358979323846  /* pi */

class Position
{
  private: //Declaration of pubs and subs

  ros::NodeHandle po_nh;
  ros::Subscriber sub;
  ros::Publisher pub;


  public:
  Position(ros::NodeHandle ao_nh) : //Constructor of the position class
    po_nh( ao_nh )
  {
    pub = po_nh.advertise<object_detector::states>( "/states", 10) ; //Publisher del tipo object_detector/states que publica al topic states
    sub = po_nh.subscribe("/corners", 10, &Position::centCalculator, this); //Suscriber del tipo object_detector/corners que se suscribe a /corners
  }

  void centCalculator(const object_detector::corners& msg) //Callback para el subscriber "CentCalculator"
  {	
        object_detector::states info; //Creacion del objeto tipo corners  para manejar la info

	
	//Calculo de el Angulo Theta
	float ABx = ((msg.BottomRightX - msg.BottomLeftX)); //Calculo ancho inferior de la imagen (X)
	float ABy = ((msg.BottomRightY - msg.BottomLeftY)); //Calculo ancho inferior de la imagen (Y)
	float theta = atan2(ABy, ABx)*180/M_PI; //Calculo de Theta con Atan2 y conversion a grados

	float vx = ABx; //Vector entre C y D para W (Inferior)
	float vy = ABy;
	float v1 = sqrt(abs(vx*vx)+abs(vy*vy)); //Calcula de la magnitud neta del vector de la esquina inferior

	vx = msg.TopRightX - msg.BottomRightX;  //Vector entre B y D para H (Izquierda)
	vy = msg.TopRightY - msg.BottomRightY;
	float v2 = sqrt(abs(vx*vx)+abs(vy*vy)); //Calcula de la magnitud neta del vector de la esquina izquierdac
			
	vx = msg.TopRightX - msg.TopLeftX; //Vector entre A y C para W (Superior)
	vy = msg.TopRightY - msg.TopLeftY; 
	float v3 = sqrt(abs(vx*vx)+abs(vy*vy)); //Calcula de la magnitud neta del vector de la esquina superior

	vx = msg.BottomLeftX - msg.TopLeftX;  //Vector entre C y A para H (Inferior)
	vy = msg.BottomLeftY - msg.TopLeftY;
	float v4 = sqrt(abs(vx*vx)+abs(vy*vy)); //Calcula de la magnitud neta del vector de la esquina inferior
		

	//Asignacion de los datos al custom msg para ser publicado
	info.Xc = msg.CenterX;
	info.Yc = msg.CenterY;
	//Caculo centroide basado en el escalamiento realizado por la homografia
	info.W = (v1+v3)/2; //Promedio realizado entre los vectores calculador para computar Width
	info.H = (v2+v4)/2; //Promedio realizado entre los vectores calculador para computar Height
	info.Theta = theta;

	if(theta>90) //Corecion de theta cuando el angulo sea mayor a 90 grados, para no generar problema en los datos publicados
	{	int cont = theta/90;
		info.Theta = theta-cont*90;
	}
	else if(theta<90)
	{	int cont = -1*theta/90;
		info.Theta = theta+cont*90;
	}
	
	//Uncomment these lines to print the results on console
	/*printf("Centroid in pix position (%f,%f)\n Object Width (%f)\n Object Height (%f)\n Theta (%f)\n", //Impresion en consola de los datos de posicion
					
			info.Xc, info.Yc,
			info.W, info.H,
			info.Theta);
	*/
	pub.publish(info);

        
  }


};//End of class 


int main(int argc, char** argv)
{   

    ros::init(argc, argv, "data_calculation"); //Nombre del nodo que uso para suscribirme y publicar la info
    ros::NodeHandle n;
    Position detec(n);


    ros::spin();

    return 0;
}
