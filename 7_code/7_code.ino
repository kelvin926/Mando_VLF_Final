// ---------------카메라 초기 설정 시작---------------
#define AOpin  A0 // Analog output - 카메라 데이터 통신
#define SIpin  11 // Start Integration 디지털 11핀 - 데이터 전송을 알리는 핀
#define CLKpin 12  // Clock 디지털 12핀 - 데이터 전송을 위한 속도(클럭) 펄스를 주는 핀
#define NPIXELS 128 // 한 배열당 몇 픽셀을 넣을 것인지 정의 
byte Pixel[NPIXELS]; // 얼마나 측정할 것인지. <0-255> (블랙 0, 흰색 255)
int LineSensor_Data_Adaption[NPIXELS]; // 보정된 라인센서 데이터 배열
int MAX_LineSensor_Data; // 센서의 최대값 정의
int MIN_LineSensor_Data; // 센서의 최소값 정의
int flag_line_adapation; // 라인센서 확인을 위한 플래그 변수
#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// ------------------ 카메라 초기 설정 끝 --------------------------------


// ------------------ 라인 센싱 초기 설정 시작 --------------------------------
void line_adaptation(void) // 라인 보정부
{
    int i;
    for (i = 0; i < NPIXELS; i++)
    {
        if (LineSensor_Data_Adaption[i] >= MAX_LineSensor_Data)  
            MAX_LineSensor_Data = LineSensor_Data_Adaption[i]; 
        //센서 데이터가 이전의 최대 데이터보다 클 경우, 최대 데이터를 센서 데이터로 입력
        if (LineSensor_Data_Adaption[i] <= MIN_LineSensor_Data)
            MIN_LineSensor_Data = LineSensor_Data_Adaption[i]; 
        //센서 데이터가 이전의 최소 데이터보다 작을 경우, 최소 데이터를 센서 데이터로 입력
    }
}

void read_line_sensor(void) //라인 센싱부
{
    int i;
    delayMicroseconds (1);  // 1ms만큼 일시중지
    delay(10);              // 10ms만큼 일시중지 -> 근데 왜 이렇게 만든거지...? 일단 건들지 않겠으.
    digitalWrite (CLKpin, LOW); // CLK핀 대기상태 - 송수신 주기 X
    digitalWrite (SIpin, HIGH); // SI핀 작동상태 - 카메라 송수신 시작.
    digitalWrite (CLKpin, HIGH); // CLK핀 작동상태 - 카메라 송수신 주기 동기화 On
    digitalWrite (SIpin, LOW); //SI핀 대기상태 - 카메라 송수신 중단.
    delayMicroseconds (1); //1ms 대기

    for (i = 0; i < NPIXELS; i++) // 원본 배열에 넣는 for문
    {
        Pixel[i] = analogRead (AOpin) / 4 ; // 8-bit is enough - 픽셀배열에 각 이미지 픽셀 값을 넣음 // 원본 데이터 입니다.
        digitalWrite (CLKpin, LOW); // CLK핀 대기상태 - 데이터 동기화 중지 (위 코드에서 High인 상태에서 해당 반복문이 시작되기에, 첫 픽셀은 기본으로 들어감.)
        delayMicroseconds (1); // 1ms대기
        digitalWrite (CLKpin, HIGH); //CLK핀 작동상태 - 데이터 동기화 시작 (이게, 데이터가 많으니까 한 픽셀에 들어갈 양만 clk값에 따라 딱 넣고 멈췄다가 다음 픽셀에 넣고 멈췄다가 이런 느낌인가봐.)
    }
    
    for (i = 0; i < NPIXELS; i++) // 0 ~ 1023까지의 데이터를 2번 과정을 거친 뒤에 -> 0 ~ 256
    {
        LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data, MAX_LineSensor_Data, 0, 256);
    }
}
// ------------------ 라인 센싱 초기 설정 끝 --------------------------------



// ------------------ DC모터 초기 설정 시작 --------------------------------
#define MOTOR_DIR 4 //노란친구(Encoder A) - D2(INT4) (방향 제어 - direction)
#define MOTOR_PWM 5 //흰색친구(Encoder B) - D3(INT5) (속도 제어 - speed (pwm))

int Motor_Speed = 0; //모터 속도 : 0 (기본값)
#define NORMAL_SPEED 100 //일반 모터 속도 : 100
#define SLOW_SPEED   70 //느린 모터 속도 : 70

void motor_control(int direction, int speed) //dc모터 컨트롤 함수 생성 
{
    digitalWrite(MOTOR_DIR, 1-direction);   //방향 값을 디지털 값으로 모터에 입력
    analogWrite(MOTOR_PWM, speed); //속도 값을 아날로그 값으로 모터의 입력
}
// ------------------ DC모터 초기 설정 끝 --------------------------------



// -------------------------------- 서보모터 초기 설정 시작 --------------------------------
#include <Servo.h> //Servo라이브러리 아두이노 프로그램에 설치해야합니당 아마 기본으로 깔려있을껄..?
#define RC_SERVO_PIN 12 //12번핀 할당
#define NEURAL_ANGLE 90 //기본 앵글: 115도 -> 전방 방향
#define LEFT_STEER_ANGLE -30 //좌측 스티어링 각도 지정
#define RIGHT_STEER_ANGLE 30 //우측 스티어링 각도 지정
Servo Steeringservo; //서보를 사용하는 함수를 미리 선언

void steering_control(int steer_angle) //앞바퀴 스티어링 함수.
{
    Steeringservo.write(NEURAL_ANGLE + steer_angle);  // 기본값(90도)에 앵글 값 대입하여 스티어링 작동 
}   
// -------------------------------- 서보모터 초기 설정 끝 --------------------------------



// -------------------------------- 이진화 시스템 시작 --------------------------------
#define threshold_value 150
void threshold(void)
{
    int i;
    for (i = 0; i < NPIXELS; i++)
    { //세부 내용은 라인센싱부 주석을 참고하세유
        if((byte)Pixel[i] >= threshold_value)
            LineSensor_Data_Adaption[i] = 255;
        else
            LineSensor_Data_Adaption[i] = 0;
    }
}
// -------------------------------- 이진화 시스템 끝 --------------------------------



// 루프에 while문을 돌려서 안전수(두 라인 검출 시스템 on/off)를 확보하고, 라인 검출에 실패했을 때 미로 탐색 함수 작동, 한 라인 검출 실패 -> 한줄모드

// ------------------ 적응형 라인 검출 시스템 시작 --------------------------------
void Two_Line(void)
{
    int i;
    int left, right; //좌,우 라인 내부 값
    int center;
    int two_steer_data = 0;
    for (i = 0; i < NPIXELS; i++) { // 왼쪽부터 순차적으로 올라갈 때 처음으로 라인이 존재하는 구간 검출
        if (LineSensor_Data_Adaption[i] == 255) {
            left = i;
        }
    }
    for (i = (NPIXELS-1); i >= 0; i--) { // 오른쪽부터 순차적으로 내려올 때 처음으로 라인이 존재하는 구간 검출
        if (LineSensor_Data_Adaption[i] == 255) {
            right = i;
        }
    }


    Serial.print("왼쪽 값은 ");
    Serial.println(left);
    Serial.print("오른쪽 값은 ");
    Serial.println(right);
    center = ((left + right)/2);
    two_steer_data = center - 64;
    int alpha = 15;               // 라인 한줄의 왼쪽 오른쪽 폭 길이
    int beta = 70;               // 라인과 라인 사이의 검은 공간의 폭 길이
    int point = (right - left) * 100 /127;             

    if (right - left <= alpha) {                                      // 15는 어림잡아서 라인 한줄의 왼쪽 오른쪽 폭 길이이다
        if (point > 50) {
            if (center * 100 /127 > 25) {
                two_steer_data = center - 64 + (alpha/2) + (beta/2);
            }

            if (center * 100 /127 < 25) {
                two_steer_data = center - 64 - (alpha/2) - (beta/2);
            }
        }

        if (point < 50) {
            if (center * 100 /127 < 75) {
                two_steer_data = center - 64 + (alpha/2) + (beta/2);
            }

            if (center * 100 /127 > 75) {
                two_steer_data = center - 64 - (alpha/2) - (beta/2);
            }
        }
    }
    Serial.println(two_steer_data);
    steering_control(two_steer_data*5);
}
// ------------------ 적응형 라인 검출 시스템 끝 ------------------------------





// -------------------------------- 초음파 시스템 시작 --------------------------------
#define DEBUG 1 // 디버깅 코드는 왜있을까~~~~
#include <NewPing.h> //초음파 관련 헤더파일임! (설치 필요)
#define SONAR_NUM 3 //초음파 센서 갯수를 부여 (3번)
#define MAX_DISTANCE 150 //cm단위 최대 값 (초음파 최대값)
float UltrasonicSensorData[SONAR_NUM];//센서 데이터 1차배열 생성(실수)

NewPing sonar[SONAR_NUM] = {   // 초음파센서 배열 생성 
    NewPing(8, 9, MAX_DISTANCE),  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정) 앞
    NewPing(2, 3, MAX_DISTANCE),  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정) 왼쪽
    NewPing(10, 11, MAX_DISTANCE)  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정) 오른쪽
};

void read_ultrasonic_sensor(void) //초음파 값 읽어들이는 함수 
{
    for(int i=0; i<=2; i++){
        UltrasonicSensorData[i]= sonar[i].ping_cm(); //해당 배열에 첫번째 초음파 센서를 이용한 cm단위의 값을 넣음
        if(UltrasonicSensorData[i] == 0.0) //0.0을 넣었을 경우(완전 붙었다기보단 너무 멀어져서 값이 0이 된 것.)
        {
            UltrasonicSensorData[i] = (float)MAX_DISTANCE; //측정값이 매우 커서 0이 나왔을 경우, 넣을 수 있는 최대값을 넣는다.
        }
    }
}
// -------------------------------- 초음파 시스템 끝 --------------------------------



// -------------------------------- 셋업 START --------------------------------
void setup() {
    // -------------------------------- 카메라/라인센싱 셋업 부분 시작 ------------------------------
    int i;
    for (i = 0; i < NPIXELS; i++) // NPIXELS = 128임.
    {
        LineSensor_Data_Adaption[i] = 0; //이미지를 나타내는 전체 픽셀들 보정 데이터를 0으로 초기화
        MAX_LineSensor_Data = 1023; //최대값을 1023으로 초기화
        MIN_LineSensor_Data = 0; //최소값을 0으로 초기화
    }
    pinMode(SIpin, OUTPUT); //카메라 SI(카메라 시작) 디지털 핀 출력모드
    pinMode(CLKpin, OUTPUT); //카메라 CLK(카메라 데이터 전송 주기 동기화) 디지털 핀 출력모드
    pinMode (AOpin, INPUT); //카메라 A0 아날로그 핀 입력모드
    digitalWrite(SIpin, LOW);   // IDLE(대기) 상태 - 카메라 데이터 송수신X
    digitalWrite(CLKpin, LOW);  // IDLE(대기) 상태 - 카메라 데이터 송수신이 없기에, 주기도 없음.
#if FASTADC
    sbi(ADCSRA, ADPS2); //위에 어샘블리 쪽이랑 관련된 것 같은데, 입력되는 데이터를 고속화하는 느낌..? 한번에 많이 가져오는건가..
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0); // 고속화 주파수에 맞게 아날로그 데이터를 디지털 주파수화 하는 느낌......
#endif
    flag_line_adapation = 0; // 얜 도대체 어디에 쓰는걸까..?
// -------------------------------- 카메라/라인센싱 셋업 부분 끝 --------------------------------
// -------------------------------- 서보모터 셋업 부분 시작 --------------------------------
    Steeringservo.attach(RC_SERVO_PIN); //사용할 핀 먼저 선언
    Steeringservo.write(NEURAL_ANGLE); //선언한 핀에 기본값 지정(전방)
// -------------------------------- 서보모터 셋업 부분 끝 --------------------------------
// -------------------------------- DC모터 셋업 부분 시작 --------------------------------
    pinMode(MOTOR_DIR, OUTPUT); //DC모터 방향 핀을 디지털 출력 핀으로 지정
    pinMode(MOTOR_PWM, OUTPUT); //DC모터 속도 핀을 디지털 출력 핀으로 지정
// -------------------------------- DC모터 셋업 부분 끝 --------------------------------
    Serial.begin(115200); // 115200속도로 시리얼 전송~
}
// -------------------------------- 셋업 END --------------------------------


// -------------------------------- 루프 START --------------------------------
void loop() {

    int i;
    read_line_sensor(); //라인 센싱부 작동~ -> 보정한 데이터 얻음.
    motor_control(0, 70); //1의 방향으로 50의 속도만큼
    threshold(); // 이진화 함수
    Two_Line();
}
// -------------------------------- 루프 END --------------------------------