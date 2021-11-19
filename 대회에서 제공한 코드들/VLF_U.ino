
#define DEBUG 1
/////////////////////  Ultrasonic Sensor ///////////////////
#include <NewPing.h>
#define SONAR_NUM 1
#define MAX_DISTANCE 150 //cm
float UltrasonicSensorData[SONAR_NUM];

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(9, 10, MAX_DISTANCE)  
};
/////////////////////  Ultrasonic Sensor ///////////////////



#define MOTOR_DIR 4
#define MOTOR_PWM 5

/////////////////////  Steering Servo Control ///////////////////
#include <Servo.h>
#define RC_SERVO_PIN 8
#define NEURAL_ANGLE 90
#define LEFT_STEER_ANLGE -30
#define RIGHT_STEER_ANLGE 30

Servo Steeringservo;

int Steering_Angle = NEURAL_ANGLE; 


void serial_com(void)
{
  if(DEBUG != 1) return;

  Serial.print("Sonar : "); Serial.println(UltrasonicSensorData[0]);
}
void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
 pinMode(MOTOR_DIR, OUTPUT);
 pinMode(MOTOR_PWM, OUTPUT);
 Steeringservo.attach(RC_SERVO_PIN);
 Steeringservo.write(NEURAL_ANGLE);

 delay(2000); 
}
void read_ultrasonic_sensor(void)
{
   UltrasonicSensorData[0]= sonar[0].ping_cm();   
}

void steering_control(int steer_angle)
{

  if(steer_angle>=RIGHT_STEER_ANLGE) steer_angle = RIGHT_STEER_ANLGE;
  if(steer_angle<=LEFT_STEER_ANLGE)  steer_angle = LEFT_STEER_ANLGE;
  Steeringservo.write(NEURAL_ANGLE + steer_angle);  
}

/////////////////////// DC Motor Control /////////////////////

#define MOTOR_PWM 5 
#define MOTOR_DIR 4
int Motor_Speed =0;
#define NORMAL_SPEED 100
#define SLOW_SPEED   70

void motor_control(int direction, int speed)
{
  digitalWrite(MOTOR_DIR, 1-direction);   //digitalWrite(MOTOR_DIR, 0);
  analogWrite(MOTOR_PWM, speed);  
}


void loop() {
  // put your main code here, to run repeatedly:
 
  read_ultrasonic_sensor();
  serial_com();
}

 

  

 

 

 

 
 

 
