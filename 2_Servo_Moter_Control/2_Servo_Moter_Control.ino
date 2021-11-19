#define MOTOR_DIR 4 //노란친구(Encoder A) - D2(INT4) (방향 제어 - direction)
#define MOTOR_PWM 5 //흰색친구(Encoder B) - D3(INT5) (속도 제어 - speed (pwm))

///////////////// 스티어링 서보모터 컨트롤부 /////////////
#include <Servo.h> //Servo라이브러리 아두이노 프로그램에 설치해야합니당 아마 기본으로 깔려있을껄..?
#define RC_SERVO_PIN 8 //8번핀 할당
#define NEURAL_ANGLE 90 //기본 앵글: 90도 -> 전방 방향
#define LEFT_STEER_ANGLE - 30 //좌측 스티어링 각도 지정
#define RIGHT_STEER_ANGLE 30 //우측 스티어링 각도 지정

Servo Steeringservo; //서보를 사용하는 함수를 미리 선언
int Steering_Angle = NEURAL_ANGLE;//기본 스티어링 값 기본값으로 지정(전방)

///////////////// 스티어링 서보모터 컨트롤부 끝 /////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //통신 속도 115200
  pinMode(MOTOR_DIR, OUTPUT); //해당 핀들을 출력 핀으로 지정
  pinMode(MOTOR_PWM, OUTPUT); //해당 핀들을 출력 핀으로 지정
  
///////////////// 스티어링 서보모터 컨트롤부 /////////////
  Steeringservo.attach(RC_SERVO_PIN); //사용할 핀 먼저 선언
  Steeringservo.write(NEURAL_ANGLE); //선언한 핀에 기본값 지정(전방)

}

void steering_control(int steer_angle) //스티어링 해주는 함수 구현
{
  Steeringservo.write(NEURAL_ANGLE + steer_angle); //위에 선언한 핀(8핀)에 원하는 스티어링 값을 입력함. (기본값(90도)에서 조정하는 방법으로 스티어링 함.)
}
///////////////// 스티어링 서보모터 컨트롤부 끝 /////////////

void motor_control(int direction, int speed) //방향값, 속도값을 인수로 받는 함수
{
  digitalWrite(MOTOR_DIR, direction); //방향 값을 디지털 값으로 모터에 입력
  analogWrite(MOTOR_PWM, speed); //속도 값을 아날로그 값으로 모터의 입력
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Motor Control");
//  int i;
//  for(i=0; i<255; i++)
//  {
//    Serial.println(i); //실행시키면 값이 차례로 올라가유~
//    motor_control(1,i); //실행시키면 i의 값이 올라갈수록 점점 빨라져유 (방향값: 1이면 앞으로, 0이면 뒤로감.)
//    delay(100); //0.1초씩 지연 
//  }
///////////////// 스티어링 서보모터 컨트롤부 /////////////
  steering_control(LEFT_STEER_ANGLE); //왼쪽으로~~
  delay(1000);
  steering_control(RIGHT_STEER_ANGLE); //오른쪽으로~~
  delay(1000);
///////////////// 스티어링 서보모터 컨트롤부 끝 /////////////
}
