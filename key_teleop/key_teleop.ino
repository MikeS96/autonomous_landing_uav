#define USE_USBCON

//All The ROS includes
#include <ros.h>
//Agrego "Libreria " que cree con el Custom msg
//#include <custos_msgs/Velocidades.h>
//el topic cmd_vel, es del tipo geometry_msgs/Twist
#include <geometry_msgs/Twist.h>
//Para hacer la cinematica inversa
#include <MatrixMath.h>
//Control de los motores
#include <Herkulex.h>
//Control de hilos en arduino
#include <ThreadController.h>
#include <Thread.h>
//Set the arduino hardware
#include <ArduinoHardware.h>

//Define hercules MotorIDs
#define DRIVE_A 50
#define DRIVE_B 51
#define DRIVE_C 52
#define DRIVE_D 53
#define STEER_A 0
#define STEER_B 3//cambie los motores
#define STEER_C 1
#define STEER_D 2
//tuneable motor parameter
#define GOAL_TIME_SPEED 10//10
#define GOAL_TIME_ANGLE 1000//500

//Variables
const double length1 = 0.6; //Largo
const double width = 0.375; //Ancho
double distZtoWheel[4]; //Distancia desde el origen a el centro de cada rueda
double wheelVel[4]; //Variable que maneja la velocidad de las llantas
double beta[4]; //Variable para almacenar los betha
double steer[4]; //Aqui almaceno el angulo S de cada llanta
double alpha[4]; //Variable para almacenar los alpha
double wheelZeroVec[4][2]; //Catetos y,x de cada llanta
float cmdangA , cmdangB; //Es el Steer pero en grados
float jac2[8][1]; //Es la Matriz J2 con las velocidades*radio llanta
float wheelrad; //radio de la llanta
float epsilon[3]; //Matriz para almacenar velocidades en el marco local

//Crea los Threads
Thread MotorControl = Thread();
//Thread MotorStatus = Thread();

ros::NodeHandle nh;

//Creo un objeto del paquete custos_msgs tipo Velocidades llamado velo
//custos_msgs::Velocidades velo;
//Creo el publisher llamado pub, y publico en un topico llamado josefina
//ros::Publisher pub("anastacia_miguelon", &velo);

void velCallback( const geometry_msgs::Twist& vel) {

  
  epsilon[0] = vel.linear.x;
  epsilon[1] = 0;
  epsilon[2] = vel.angular.z; //velocidad angular theta punto

  float radioP = (epsilon[0] / epsilon[2]); //Esto es ICRY

  if (epsilon[0] == 0 && epsilon[2] > 0) { //Esta conficion limita el movimiento solo de las llantas a la izquierda sin velocidad en X
    radioP = 0.5;
  }

  if (epsilon[0] == 0 && epsilon[2] < 0) { //Esta conficion limita el movimiento solo de las llantas a la derecha sin velocidad en X
    radioP = -0.5;
  }

  float steering = tan(length1 / (radioP)); //Steer rueda virtual

  if (epsilon[2] == 0) {
    steering = 0; //Esta condicion evita errores en el codigo, ya que tan(a/0) genera error
    radioP = 0;
  }

  float angA, angB;
  //Estos if hacen la actualizacion de los angulos S.
  if (steering > 0) {   //
    angA = (atan2(length1, (radioP + (width / 2))));
    angB = (atan2(length1, (radioP - (width / 2))));
    steer[0] = angA;
    steer[1] = angB;
    steer[2] = 0;
    steer[3] = 0;
  } else if (steering < 0) {
    angA = (atan2(length1, (abs(radioP) - (width / 2)))); //Se usa valor absoluto para tomar la distancia (0,ICRy) y no sunubicación en el planof
    angB = (atan2(length1, (abs(radioP) + (width / 2))));
    steer[0] = -angA;
    steer[1] = -angB;
    steer[2] = 0;
    steer[3] = 0;
  } else {
    steer[0] = 0;
    steer[1] = 0;
    steer[2] = 0;
    steer[3] = 0;
  }


  //Condiciones beta
  for (int i = 0; i < 4; i++) {
      if((steering < 0) || i > 1 || (steering == 0)) {
        beta[i] = 1.5708 + steer[i] - alpha[i]; //Condicion con ICRY a la derecha
      }

      else if((steering > 0) && i < 2 ) {
        beta[i] = 1.5708 - steer[i] + alpha[i]; //Condicion con ICRY a la izquierda
      }
  }


  if (steering > 0) { //En caso de que el ICR este a la izquierda y x>0
    beta[0] = 3.1415 - beta[0]; //Se debe cambiar Betha haciendo phi-Betha_actual, esto debido al
    beta[1] = 3.1415 - beta[1]; //Sentido de giro de las ruedas.
  }

  //Restricciones de deslizamiento y rodamiento
  float  jac1[8][3] = {
    {sin(alpha[0] + beta[0]), cos(alpha[0] + beta[0]) * -1, -1 * distZtoWheel[0]* cos(beta[0])},
    {sin(alpha[1] + beta[1]), cos(alpha[1] + beta[1]) * -1, -1 * distZtoWheel[1] * cos(beta[1])},
    {sin(alpha[2] + beta[2]), cos(alpha[2] + beta[2]) * -1, -1 * distZtoWheel[2] * cos(beta[2])},
    {sin(alpha[3] + beta[3]), cos(alpha[3] + beta[3]) * -1, -1 * distZtoWheel[3] * cos(beta[3])},
    {cos(alpha[0] + beta[0]), sin(alpha[0] + beta[0]), distZtoWheel[0] * sin(beta[0])},
    {cos(alpha[1] + beta[1]), sin(alpha[1] + beta[1]), distZtoWheel[1] * sin(beta[1])},
    {cos(alpha[2] + beta[2]), sin(alpha[2] + beta[2]), distZtoWheel[2] * sin(beta[2])},
    {cos(alpha[3] + beta[3]), sin(alpha[3] + beta[3]), distZtoWheel[3] * sin(beta[3])}
  };

  //jac2=jac1*epsilon(local);

  //J1(8x3)*Guama(3x1) => MatrixMath.Multiply(jac1,guama,filasJ1=8,columnasJ1=filasGuama=3,columnasGuama=1,matrizResultante)
  //epsilon es guamaR, es decir el marco local del robot
  matrix.Multiply((float*)jac1, (float*)epsilon, 8, 3, 1, (float*)jac2);


  //  Convierte de RADIANES a GRADOS, los motores reciben la posiciones en grados
  cmdangA = ((steer[0]) * (180 / 3.1416));
  cmdangB = ((steer[1]) * (180 / 3.1416));

  //Se utiliza el offset 57.8499984741, que es la posicion cero fìsica de los motores
  //if(steering>0){
  steer[0] = (57.8499984741 * -1) - cmdangA; //Esta condicion solo mueve las llantas hacia la derecha...
  steer[1] = 57.8499984741 - cmdangB;
  steer[2] = 57.8499984741;
  steer[3] = 57.8499984741 * -1;


  //convertir jac2 en las velocidades de las ruedas usando wheelrad
  //se convierte de RADIANES/SEG a GRADOS/SEG para que lo entiendan los motores
  //por la ubicacion de los motores al motor A, se le multiplica -1
 
  wheelVel[0] = ((-1 * jac2[0][0] / wheelrad) * 180 / (3.1416)); //velocidades en grados sobre segundos RUEDA A
  wheelVel[1] = ((jac2[1][0] / wheelrad) * 180 / (3.1416)); //ESTA LINEA ME GENERA DUDA RUEDA B
  wheelVel[2] = ((jac2[2][0] / wheelrad) * 180 / (3.1416)); //ESTA LINEA ME GENERA DUDA RUEDA C
  wheelVel[3] = ((-1 * jac2[3][0] / wheelrad) * 180 / (3.1416)); //velocidades en grados sobre segundos RUEDA D
  
    //Convertir  la resolucion de los motores 1 unidad = 0,62 grad/seg
  wheelVel[0] = wheelVel[0] / 0.62;
  wheelVel[1] = wheelVel[1] / 0.62;
  wheelVel[2] = wheelVel[2] / 0.62;
  wheelVel[3] = wheelVel[3] / 0.62;

  if (epsilon[0] == 0) { //Esto aplica para los casos que la velocidad en X=0, para no generar problemas en el codigo
    wheelVel[0] = 0;
    wheelVel[1] = 0;
    wheelVel[2] = 0;
    wheelVel[3] = 0;
  }

}

/*
//Metodo para obtener la velocidad de los encoders
void getEncoderData() {

  double velrad0, velrad1;

  int vel0, vel1; //modificacion mapeo encoders

  vel0 = Herkulex.getSpeed(50);
  vel1 = Herkulex.getSpeed(51);

  //hacer mapeo de los encoders

  if (epsilon[0] == 0) {
    vel0 = 0;
    vel1 = 0;
  } else if (epsilon[0] < 0) {

    vel0 = vel0 * -1;
    vel1 = map(vel1, 65538, 65113, -5, -420); //map(value, fromLow, fromHigh, toLow, toHigh)

  } else if (epsilon[0] > 0) {

    vel0 = map(vel0, 65538, 65113, 5, 420); //map(value, fromLow, fromHigh, toLow, toHigh)

  }

  velo.rawVelA=(float)wheelVel[0]*0.62;
  velo.rawVelB=(float)wheelVel[1]*0.62;
  velo.rawVelC=(float)wheelVel[2]*0.62;
  velo.rawVelD=(float)wheelVel[3]*0.62;
  velo.velA=(float)vel0*0.62;
  velo.velB=(float)vel1*0.62;
}
*/

//Este metodo se encarga de enviar los datos al motor.
void sendMotorCommands() {

  Herkulex.moveOneAngle(STEER_A, steer[0], GOAL_TIME_ANGLE, LED_BLUE);
  Herkulex.moveOneAngle(STEER_B, steer[1], GOAL_TIME_ANGLE, LED_BLUE);
  //Herkulex.moveOneAngle(STEER_C, steer[2], GOAL_TIME_ANGLE, LED_BLUE);
  //Herkulex.moveOneAngle(STEER_D, steer[3], GOAL_TIME_ANGLE, LED_BLUE);

  //Speeds

  Herkulex.moveSpeedOne(DRIVE_A, wheelVel[0], GOAL_TIME_SPEED, LED_GREEN);
  Herkulex.moveSpeedOne(DRIVE_B, wheelVel[1], GOAL_TIME_SPEED, LED_GREEN);
  Herkulex.moveSpeedOne(DRIVE_C, wheelVel[2], GOAL_TIME_SPEED, LED_GREEN);
  Herkulex.moveSpeedOne(DRIVE_D, wheelVel[3], GOAL_TIME_SPEED, LED_GREEN);
}

//Aqui se instancea el publisher, se suscribe al topic cmd_vel, y es del tipo geometry_msgs::Twist, ademas esta atado al metodo veCallback
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);

void setup() {
  Serial.begin(115200);
  Herkulex.beginSerial3(115200); //Este serial maneja los motores
  delay(5000);
  Serial.print("Serial 1 y 3 Setted");

  // Cero físico de los motores
  // 1 y 2 están en el mismo sentido, 3 y 4 están en sentido contrario, por ello se multiplican por -1
  steer[2] = 57.8499984741;
  steer[3] = 57.8499984741 * -1;
  steer[0] = 57.8499984741 * -1;
  steer[1] = 57.8499984741;


  //Calculo de catetos para cada llanta
  wheelZeroVec[0][0] = length1;
  wheelZeroVec[0][1] = -width / 2;
  wheelZeroVec[1][0] = length1;
  wheelZeroVec[1][1] = width / 2;
  wheelZeroVec[2][0] = 0;
  wheelZeroVec[2][1] = width / 2;
  wheelZeroVec[3][0] = 0;
  wheelZeroVec[3][1] = -width / 2;

  //Calculo de la distancia desde el marco de referencia al centro de cada llanta
  distZtoWheel[0] = sqrt((pow(wheelZeroVec[0][0], 2)) + (pow(wheelZeroVec[0][1], 2)));
  distZtoWheel[1] = sqrt((pow(wheelZeroVec[1][0], 2)) + (pow(wheelZeroVec[1][1], 2)));
  distZtoWheel[2] = sqrt((pow(wheelZeroVec[2][0], 2)) + (pow(wheelZeroVec[2][1], 2)));
  distZtoWheel[3] = sqrt((pow(wheelZeroVec[3][0], 2)) + (pow(wheelZeroVec[3][1], 2)));

  wheelrad = 0.076; //Radio de la llanta del vehiculo
  for (int i = 0; i < 4; i++) {

    wheelVel[i] = 0; //Inicializa en 0 las velocidades de los motores

    //Calculo de los alpha de cada rueda
    alpha[i] = atan2(wheelZeroVec[i][1], wheelZeroVec[i][0]);


  }
  nh.getHardware()->setBaud(115200); //Setea el baudeaje al que se trabajara con ROS
  nh.initNode();  //Inicia ROS, permite crear publisher and suscribers y atiende la comunicacion seria;
  nh.subscribe(sub); //Me suscribo
  //nh.advertise(pub); //Publico

  //Inicializacion de los motores
  Herkulex.reboot(STEER_A);
  Herkulex.reboot(STEER_B);
  //Herkulex.reboot(STEER_C);
  //Herkulex.reboot(STEER_D);
  Herkulex.reboot(DRIVE_A);
  Herkulex.reboot(DRIVE_B);
  Herkulex.reboot(DRIVE_C);
  Herkulex.reboot(DRIVE_D);
  delay(500);
  Herkulex.initialize();

  //Metodo para enviar datos a los motores
  MotorControl.onRun(sendMotorCommands);
  MotorControl.setInterval(200);
  //Metodo para leer los encoders
  //MotorStatus.onRun(getEncoderData);
  //MotorStatus.setInterval (300);

}

void loop() {
  if ( MotorControl.shouldRun()) {
    MotorControl.run();
  }
  /*
  if (MotorStatus.shouldRun()) {
    MotorStatus.run();
    //send data
    pub.publish(&velo); //Con esto publico la informacion que contenga velo en el topico anastasia_miguelon
    nh.spinOnce(); //Con el spinOnce todas las comunicaciones de ROS son manejados (Handleded)
  }
  */
  nh.spinOnce();


}
