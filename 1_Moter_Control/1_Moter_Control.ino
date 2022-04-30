#define MOTOR_DIR 4 //노란친구(Encoder A) - D2(INT4) (방향 제어 - direction)
#define MOTOR_PWM 5 //흰색친구(Encoder B) - D3(INT5) (속도 제어 - speed (pwm))

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //통신 속도 115200
  pinMode(MOTOR_DIR, OUTPUT); //해당 핀들을 출력 핀으로 지정
  pinMode(MOTOR_PWM, OUTPUT); //해당 핀들을 출력 핀으로 지정
}

void motor_control(int direction, int speed) //방향값, 속도값을 인수로 받는 함수
{
  digitalWrite(MOTOR_DIR, direction); //방향 값을 디지털 값으로 모터에 입력
  analogWrite(MOTOR_PWM, speed); //속도 값을 아날로그 값으로 모터의 입력
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Motor Control\n");
  int i;
  for(i=0; i<255; i++)
  {
    Serial.println(i); //실행시키면 값이 차례로 올라가유~
    motor_control(1,i); //실행시키면 i의 값이 올라갈수록 점점 빨라져유 (방향값: 1이면 앞으로, 0이면 뒤로감.)
    delay(100); //0.1초씩 지연 ms
  }

}
