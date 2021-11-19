

///////////////////////////////////
//newping 라이브러리 설치 필요!!!
///////////////////////////////////

#define DEBUG 1
/////////////////////  초음파  ///////////////////
#include <NewPing.h> //초음파 관련 헤더파일임!
#define SONAR_NUM 1 //초음파 센서 번호를 부여 (1번)
#define MAX_DISTANCE 150 //cm단위 최대 값 (초음파 최대값)
float UltrasonicSensorData[SONAR_NUM];//센서 데이터 1차배열 생성(실수)

NewPing sonar[SONAR_NUM] = {   // 초음파센서 배열 생성 
  NewPing(9, 10, MAX_DISTANCE)  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정)
};
/////////////////////  초음파 끝 ///////////////////


/////////////////////  서보모터 ///////////////////
#include <Servo.h> //Servo라이브러리 아두이노 프로그램에 설치해야합니당 아마 기본으로 깔려있을껄..?
#define RC_SERVO_PIN 8 //8번핀 할당
#define NEURAL_ANGLE 90 //기본 앵글: 90도 -> 전방 방향
#define LEFT_STEER_ANGLE - 30 //좌측 스티어링 각도 지정
#define RIGHT_STEER_ANGLE 30 //우측 스티어링 각도 지정

Servo Steeringservo; //서보를 사용하는 함수를 미리 선언
int Steering_Angle = NEURAL_ANGLE;//기본 스티어링 값 기본값으로 지정(전방)
///////////////// 스티어링 서보모터 컨트롤부 끝 /////////////


/////////////////////  초음파 ///////////////////
void serial_com(void)
{
  if(DEBUG != 1) return;

  Serial.print("Sonar : "); Serial.println(UltrasonicSensorData[0]);
}
/////////////////////  초음파 끝 ///////////////////


void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200); //통신 속도 115200
 pinMode(MOTOR_DIR, OUTPUT); //해당 핀들을 출력 핀으로 지정 [서보모터]
 pinMode(MOTOR_PWM, OUTPUT); //해당 핀들을 출력 핀으로 지정 [서보모터]
 
///////////////// 스티어링 서보모터 컨트롤부 /////////////
  Steeringservo.attach(RC_SERVO_PIN); //사용할 핀 먼저 선언
  Steeringservo.write(NEURAL_ANGLE); //선언한 핀에 기본값 지정(전방)

 delay(2000); //2초 쉬자~
}

///////////////// 초음파  //////////////
void read_ultrasonic_sensor(void) //초음파 값 읽어들이는 함수 
{
   UltrasonicSensorData[0]= sonar[0].ping_cm(); //해당 배열에 첫번째 초음파 센서를 이용한 cm단위의 값을 넣음

   if(UltrasonicSensorData[0] == 0.0) //0.0을 넣었을 경우(완전 붙었다기보단 너무 멀어져서 값이 0이 된 것.)
   {
    UltrasonicSensorData[0] = (float)MAX_DISTANCE; //측정값이 매우 커서 0이 나왔을 경우, 넣을 수 있는 최대값을 넣는다.
   }
}
///////////////// 초음파 끝 ///////////////

void steering_control(int steer_angle) //앞바퀴 스티어링 함수.
{

  if(steer_angle>=RIGHT_STEER_ANLGE) steer_angle = RIGHT_STEER_ANLGE; //스티어링 앵글 값이 오른쪽 스티어링 값 이상일 경우, 앵글에 오른쪽 스티어링 값 넣음 
  if(steer_angle<=LEFT_STEER_ANLGE)  steer_angle = LEFT_STEER_ANLGE; //이 코드도 위랑 비슷하게 작동
  Steeringservo.write(NEURAL_ANGLE + steer_angle);  // 기본값(90도)에 앵글 값 대입하여 스티어링 작동 
}

/////////////////////  DC모터 ///////////////////
#define MOTOR_DIR 4 //노란친구(Encoder A) - D2(INT4) (방향 제어 - direction)
#define MOTOR_PWM 5 //흰색친구(Encoder B) - D3(INT5) (속도 제어 - speed (pwm))
int Motor_Speed =0; //모터 속도 : 0 (기본값)
#define NORMAL_SPEED 100 //일반 모터 속도 : 100
#define SLOW_SPEED   70 //느린 모터 속도 : 70

void motor_control(int direction, int speed) //dc모터 컨트롤 함수 생성 
{
  digitalWrite(MOTOR_DIR, 1-direction);   //방향 값을 디지털 값으로 모터에 입력
  analogWrite(MOTOR_PWM, speed); //속도 값을 아날로그 값으로 모터의 입력
}


void loop() {
  // put your main code here, to run repeatedly:
 
  read_ultrasonic_sensor(); //초음파 센서 값 리딩 
  serial_com(); //시리얼에 계속 띄움.
}

 

  

 

 

 

 
 

 
