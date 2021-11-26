// ---------------카메라 초기 설정 시작---------------
#define AOpin  A0 // Analog output - 카메라 데이터 통신
#define SIpin  11 // Start Integration 디지털 11핀 - 데이터 전송을 알리는 핀
#define CLKpin 12  // Clock 디지털 12핀 - 데이터 전송을 위한 속도(클럭) 펄스를 주는 핀
#define NPIXELS 128 // 해상도 : 가로로 128
byte Pixel[NPIXELS]; // 픽셀 배열 : 128개의 배열 만듦
int LineSensor_Data_Adaption[NPIXELS]; // 보정된 라인센서 데이터 배열
int MAX_LineSensor_Data; // 센서의 최대값 정의
int MIN_LineSensor_Data; // 센서의 최소값 정의
int flag_line_adapation; // 라인트레이싱 on/off (1:on, 0:off) - 곧 구현 예정
#define FASTADC 1 //아날로그 리딩을 가속화 시키는 FASTADC 변수 정의 (카메라 값을 빠르게 받아오기 위함.)
                    // 레지스터 비트 설정 및 소거에 대한 정의 [가속화 관련]
                    // http://electronics.arunsworld.com/basic-avr/ -> 관련 레퍼런스 첨부합니당
                    // https://github.com/empierre/arduino/blob/master/TSL1401 -> 깃헙 레퍼런스 첨부합니다 (21.11.26)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // 임시 레지스트 삭제.
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// ------------------ 카메라 초기 설정 끝 --------------------------------



// ------------------ 라인 센싱 초기 설정 시작 --------------------------------

void read_line_sensor(void) //라인 센싱부
{
    int i;
    delayMicroseconds (1);
    delay(10); // 의도된 것. -> TSL1401 datasheet 3페이지 1번 노트 참고.
    digitalWrite (CLKpin, LOW); // CLK핀 대기상태 - 송수신 주기 X
    digitalWrite (SIpin, HIGH); // SI핀 작동상태 - 카메라 송수신 시작.
    digitalWrite (CLKpin, HIGH); // CLK핀 작동상태 - 카메라 송수신 주기 동기화 On
    digitalWrite (SIpin, LOW); //SI핀 대기상태 - 카메라 송수신 중단.
    delayMicroseconds (1);
// 이 시점에서 실시간 이미지 리딩을 완료함.
    for (i = 0; i < NPIXELS; i++)
    {
        Pixel[i] = analogRead(AOpin)/4; // 2^8=256->8bit 체계를 만들어줌. (이미지 배열에 탑재)
        digitalWrite (CLKpin, LOW); // CLK핀 대기상태 - 데이터 동기화 중지 (위 코드에서 High인 상태에서 해당 반복문이 시작되기에, 첫 픽셀은 기본으로 들어감.)
        delayMicroseconds (1);
        digitalWrite (CLKpin, HIGH); //CLK핀 작동상태 - 데이터 동기화 시작 (이게, 데이터가 많으니까 한 픽셀에 들어갈 양만 clk값에 따라 딱 넣고 멈췄다가 다음 픽셀에 넣고 멈췄다가 이런 느낌인가봐.)
    }

    for (i = 0; i < NPIXELS; i++)
    {
        //해당 코드는 테스트 중인 코드입니다. [시작]
        //적응형 라이닝 코드를 목표로 합니다.
        if (Pixel[i] >= MAX_LineSensor_Data){
            MAX_LineSensor_Data = Pixel[i];
        }
        if (Pixel[i] <= MIN_LineSensor_Data){
            MIN_LineSensor_Data = Pixel[i];
        }
    } // 이러면 128배열 속 가장 어두운 부분이 Min_LineSensor_Data가 되고, 가장 밝은 부분이 MAX_LineSensor_Data가 된다.
    for (i = 0; i < NPIXELS; i++)
    {
        LineSensor_Data_Adaption[i] = map(Pixel[i], 0, 255, MIN_LineSensor_Data, MAX_LineSensor_Data); //map(변환할 수, 현재 범위의 최소값, 현재 범위의 최대값, 목표 범위의 최소값, 목표 범위의 최대값)
        /* 보정데이터에 각 픽셀마다의 값을 가져와서 0~255값을 넣어서 최종적으로 0~255의 값을 갖게 함.
        // 한마디로, 카메라로 이미지를 찍었으면, 가장 흰색에 가까운 값을 최대로 두고, 가장 검은색에 가까운 값을 최소로 둠.
        // 이후 최대값을 255로 기준을 삼고, 최소값을 0으로 두어서, 상황에 따라 능동적으로 색을 분리할 수 있게함.
        // 검은색에 최대한 가까운 값을 0으로 보정, 흰색에 가장 가까운 값을 255로 고정해서 범위를 능동적으로 조절한다.
        대충 느낌 알꺼라 믿으 */
    }
        //해당 코드는 테스트 중인 코드입니다. [끝]
}
// ------------------ 라인 센싱 초기 설정 끝 --------------------------------



// ------------------ DC모터 초기 설정 시작 --------------------------------
#define MOTOR_DIR 4 //노란친구(Encoder A) - D2(INT4) (방향 제어 - direction)
#define MOTOR_PWM 5 //흰색친구(Encoder B) - D3(INT5) (속도 제어 - speed (pwm))

int Motor_Speed = 0; //모터 속도 : 0 (기본값) [정지]
#define NORMAL_SPEED 100 //일반 모터 속도 : 100
#define SLOW_SPEED   50 //느린 모터 속도 : 50

void motor_control(int direction, int speed) //dc모터 컨트롤 함수 생성 
{
    digitalWrite(MOTOR_DIR, 1-direction);   //방향 값을 디지털 값으로 모터에 입력
    analogWrite(MOTOR_PWM, speed); //속도 값을 아날로그 값으로 모터의 입력
}
// ------------------ DC모터 초기 설정 끝 --------------------------------



// -------------------------------- 서보모터 초기 설정 시작 --------------------------------
#include <Servo.h> //Servo라이브러리 아두이노 프로그램에 설치해야합니당 아마 기본으로 깔려있을껄..?
#define RC_SERVO_PIN 8 //8번핀 할당
#define NEURAL_ANGLE 90 //기본 앵글: 90도 -> 전방 방향
#define LEFT_STEER_ANGLE -40 //좌측 스티어링 각도 지정
#define RIGHT_STEER_ANGLE 40 //우측 스티어링 각도 지정
Servo SteeringServo; //서보를 사용하는 함수를 미리 선언
int Steering_Angle = NEURAL_ANGLE;//기본 스티어링 값 기본값으로 지정(전방)

void steering_control(int steer_angle) //앞바퀴 스티어링 함수.
{
    SteeringServo.write(NEURAL_ANGLE + steer_angle);  // 기본값(90도)에 앵글 값 대입하여 스티어링 작동 
    // if(steer_angle>=RIGHT_STEER_ANGLE)
    //     steer_angle = RIGHT_STEER_ANGLE; 
    //     //스티어링 앵글 값이 오른쪽 스티어링 값 이상일 경우, 앵글에 오른쪽 스티어링 값 넣음 
    // if(steer_angle<=LEFT_STEER_ANGLE)
    //     steer_angle = LEFT_STEER_ANGLE;
    //     //이 코드도 위랑 비슷하게 작동
}   
// -------------------------------- 서보모터 초기 설정 끝 --------------------------------



// -------------------------------- 이진화 시스템 시작 --------------------------------
#define threshold_value 180
void threshold(void)
{
    int i;
    for (i = 0; i < NPIXELS; i++)
    { //세부 내용은 라인센싱부 주석을 참고하세유
        if((byte)Pixel[i] >= threshold_value) // 흰색
            LineSensor_Data_Adaption[i] = 255;
        else // 검은색
            LineSensor_Data_Adaption[i] = 0;
    }
}
// -------------------------------- 이진화 시스템 끝 --------------------------------



// -------------------------------- 두 차선 인식 시스템 시작 --------------------------------
//void two_line_system(void)
//{
//    int i
//}

// -------------------------------- 두 차선 인식 시스템 끝 ----------------------------------



// -------------------------------- 센터링(무게중심) 시스템 시작 --------------------------------
#define camera_pixel_offset 0 //센터가 0이 출력되도록 보정해주는 값 (수정 필요)
void steering_by_camera(void)
{
    int i;
    long sum = 0;
    long x_sum = 0;
    int steer_data = 0;
    for (i = 0; i < NPIXELS; i++) // i -> 0~127
    {
        x_sum += LineSensor_Data_Adaption[i] * i;
        sum += LineSensor_Data_Adaption[i]; 
    } // x_sum/sum을 할 경우 데이터 중앙값의 위치(배열 i값)가 나옴.
    // 그러면 그 값에 정중앙을 빼면, 스티어링 값이 나오겠죵..?
    
    // Serial.println("----------------------");
    // Serial.print("sum은 ");
    // Serial.println(sum);
    // Serial.print("x_sum은 ");
    // Serial.println(x_sum);
    steer_data = ((x_sum/sum) - (NPIXELS/2)) + camera_pixel_offset;
    // 싱글라인 기준. 듀얼라인의 경우 위에 sum부분에서 절반을 나누어서 각 스티어링을 구해야 할듯.

    steering_control(steer_data*1); // 곱하는 값은 가중치(소수점 권장함!)

    // Serial.print("steer_data는 ");
    Serial.println(steer_data);
}
// -------------------------------- (무게중심) 시스템 끝 --------------------------------



// -------------------------------- 초음파 시스템 시작 --------------------------------
#define DEBUG 1
#include <NewPing.h> //초음파 관련 헤더파일임! (설치 필요)
#define SONAR_NUM 1 //초음파 센서 번호를 부여 (1번)
#define MAX_DISTANCE 150 //cm단위 최대 값 (초음파 최대값)
float UltrasonicSensorData[SONAR_NUM];//센서 데이터 1차배열 생성(실수)

NewPing sonar[SONAR_NUM] = {   // 초음파센서 배열 생성 
  NewPing(9, 10, MAX_DISTANCE)  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정)
};

void read_ultrasonic_sensor(void) //초음파 값 읽어들이는 함수 
{
    UltrasonicSensorData[0]= sonar[0].ping_cm(); //해당 배열에 첫번째 초음파 센서를 이용한 cm단위의 값을 넣음
    delay(10);
    if(UltrasonicSensorData[0] == 0.0) //0.0을 넣었을 경우(완전 붙었다기보단 너무 멀어져서 값이 0이 된 것.)
    {
        UltrasonicSensorData[0] = (float)MAX_DISTANCE; //측정값이 매우 커서 0이 나왔을 경우, 넣을 수 있는 최대값을 넣는다.
    }
}
// -------------------------------- 초음파 시스템 끝 --------------------------------



// -------------------------------- 셋업 START --------------------------------
void setup() {
    // -------------------------------- 카메라/라인센싱 셋업 부분 시작 ------------------------------
    pinMode(SIpin, OUTPUT); //카메라 SI(카메라 시작) 디지털 핀 출력모드
    pinMode(CLKpin, OUTPUT); //카메라 CLK(카메라 데이터 전송 주기 동기화) 디지털 핀 출력모드
    pinMode(AOpin, INPUT); //카메라 A0 아날로그 핀 입력모드
    digitalWrite(SIpin, LOW);   // IDLE(대기) 상태 - 카메라 데이터 송수신X
    digitalWrite(CLKpin, LOW);  // IDLE(대기) 상태 - 카메라 데이터 송수신이 없기에, 주기도 없음.
#if FASTADC //아날로그 고속화 리딩이 1(True) 일 때
            // 16Mhz속도로 고속화시킴. [아날로그 데이터를 디지털화 한다고 하더라구요..?!]
            // 16MHz이라면 ADC를 위한 주파수는 16M/64 = 250kHz가 된다.
            // https://studymake.tistory.com/381 - 관련 레퍼런스
    sbi(ADCSRA, ADPS2); //위에 어샘블리 쪽이랑 관련된 것 같은데, 입력되는 데이터를 고속화하는 느낌..? 한번에 많이 가져오는건가..
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0); // 고속화 주파수에 맞게 아날로그 데이터를 디지털 주파수화 하는 느낌......
#endif
    int i;
    for (i = 0; i < NPIXELS; i++) // NPIXELS = 128임.
    {
        LineSensor_Data_Adaption[i] = 0; //이미지를 나타내는 전체 픽셀들 보정 데이터를 0으로 초기화
        MAX_LineSensor_Data = -1; //함수 연계
        MIN_LineSensor_Data = 256; //함수 연계
    }
// -------------------------------- 카메라/라인센싱 셋업 부분 끝 --------------------------------


// -------------------------------- 서보모터 셋업 부분 시작 --------------------------------
    SteeringServo.attach(RC_SERVO_PIN); //사용할 핀 먼저 선언
    SteeringServo.write(NEURAL_ANGLE); //선언한 핀에 기본값 지정(전방)
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
    motor_control(0, 50); //1의 방향으로 50의 속도만큼
    read_ultrasonic_sensor(); //초음파 센서 값 리딩
    read_line_sensor(); //라인 센싱부 작동~ -> 보정한 데이터 얻음.
    threshold(); // 이진화 함수
    steering_by_camera(); //센터링(무게중심) 함수
    delay(100);
}
// -------------------------------- 루프 END --------------------------------
