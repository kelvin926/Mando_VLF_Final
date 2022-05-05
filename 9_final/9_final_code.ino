#include <Servo.h> //Servo라이브러리 아두이노 프로그램에 설치해야합니당 아마 기본으로 깔려있을껄..?
#define RC_SERVO_PIN 40 //40번핀 할당
#define NEURAL_ANGLE 90 //기본 앵글: 90도 -> 전방 방향
Servo Steeringservo;
// ---------------카메라 초기 설정 시작---------------
#define AOpin  A0 // Analog output - 카메라 데이터 통신
#define SIpin  11 // Start Integration 디지털 11핀 - 카메라 데이터 전송을 알리는 핀
#define CLKpin 12  // Clock 디지털 12핀 - 데이터 전송을 위한 속도(클럭) 펄스를 주는 핀
#define NPIXELS 128 // 해상도
byte Pixel[NPIXELS]; // 얼마나 측정할 것인지. <0-255> (블랙 0, 흰색 255)
int LineSensor_Data_Adaption[NPIXELS]; // 보정된 라인센서 데이터 배열
#define FASTADC 1 // 데이터 가속화
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
void read_ultrasonic_sensor(void);

int Line_Exist = 1; // 라인 존재 여부 (기본값 = 1)
int object_exist = 0; // 앞에 장애물 존재 여부
int right_turn = 0;

// 이진화
#define threshold_value 20
void threshold(void)
{
    int i;
    for (i = 0; i < NPIXELS; i++)
    {
        if((byte)Pixel[i] >= threshold_value)
            LineSensor_Data_Adaption[i] = 255;
        else
            LineSensor_Data_Adaption[i] = 0;
    }
}


////////////////////////////////////////////////////////////////

void read_line_sensor(void) //라인 센싱부
{
    delayMicroseconds (1);  // 1ms만큼 일시중지
    delay(10);              // 10ms만큼 일시중지 -> 근데 왜 이렇게 만든거지...? 일단 건들지 않겠으.
    digitalWrite (CLKpin, LOW); // CLK핀 대기상태 - 송수신 주기 X
    digitalWrite (SIpin, HIGH); // SI핀 작동상태 - 카메라 송수신 시작.
    digitalWrite (CLKpin, HIGH); // CLK핀 작동상태 - 카메라 송수신 주기 동기화 On
    digitalWrite (SIpin, LOW); //SI핀 대기상태 - 카메라 송수신 중단.
    delayMicroseconds (1); //1ms 대기

    int i;
    for (i = 0; i < NPIXELS; i++)
    {
        Pixel[i] = analogRead (AOpin) / 4 ; // 8-bit is enough - 픽셀배열에 각 이미지 픽셀 값을 넣음
        digitalWrite (CLKpin, LOW); // CLK핀 대기상태 - 데이터 동기화 중지 (위 코드에서 High인 상태에서 해당 반복문이 시작되기에, 첫 픽셀은 기본으로 들어감.)
        delayMicroseconds (1); // 1ms대기
        digitalWrite (CLKpin, HIGH); //CLK핀 작동상태 - 데이터 동기화 시작 (이게, 데이터가 많으니까 한 픽셀에 들어갈 양만 clk값에 따라 딱 넣고 멈췄다가 다음 픽셀에 넣고 멈췄다가 이런 느낌인가봐.)
    }
    threshold();
}


void steering_control(int steer_angle) //앞바퀴 스티어링 함수.
{
    Steeringservo.write(NEURAL_ANGLE + steer_angle);  // 기본값(90도)에 앵글 값 대입하여 스티어링 작동
}   


//************************************************************************************************

#define MOTOR_DIR 4 //노란친구(Encoder A) - D2(INT4) (방향 제어 - direction)
#define MOTOR_PWM 5 //흰색친구(Encoder B) - D3(INT5) (속도 제어 - speed (pwm))

void motor_control(int direction, int speed) //dc모터 컨트롤 함수 생성 
{
    digitalWrite(MOTOR_DIR, direction);   //방향 값을 디지털 값으로 모터에 입력
    analogWrite(MOTOR_PWM, speed); //속도 값을 아날로그 값으로 모터의 입력
}

// ------------------


void Two_Line(void)
{
    read_line_sensor();
    int i = 0, k = 0, sum = 0, left = 0, right = 0; // left,right:좌,우 라인 내부 값
    float center = 0;
    int two_steer_data = 0; // 조향값.(가중치 안더함.)
    int right_out_num, left_out_num;
    float additional_steer_num = 1.3; // 조향 가중치 - 계속 변경 가능(1:기본값)

    int kill_left = 0, kill_right = 0; // 킬스위치
    for (i = 0; i < NPIXELS; i++) {// 왼쪽부터 순차적으로 올라갈 때 처음으로 라인이 존재하는 구간 검출
        sum += LineSensor_Data_Adaption[i];
        if (LineSensor_Data_Adaption[i] == 255) {
            Line_Exist = 1;
            while(kill_left == 0) {
                left = i;
                kill_left = 1;
            }
        }
        sum += LineSensor_Data_Adaption[i];
    }
    for (i = (NPIXELS-1); i >= 0; i--) { // 오른쪽부터 순차적으로 내려올 때 처음으로 라인이 존재하는 구간 검출
        if (LineSensor_Data_Adaption[i] == 255) {
            Line_Exist = 1;
            while(kill_right == 0) {
                right = i;
                kill_right = 1;
            }
        }
    }

    if (sum == 0) { // 검출 다 돌았는데, 라인이 없을 때.
        Line_Exist = 0;
    }
    else{ // 라인이 존재한다면
        Serial.print("왼쪽 값은 ");
        Serial.println(left);
        Serial.print("오른쪽 값은 ");
        Serial.println(right);
        Line_Exist = 1;
        center = ((left + right)/2);
        two_steer_data = center - 64; // 양수면 오른쪽으로, 음수면 왼쪽으로 움직임.
        Serial.println(two_steer_data); // DEBUG
        steering_control(two_steer_data * additional_steer_num); // 가중치 : additional_steer_num -> 조향값을 더 줘야함.
    }
}


// ------------------
#define SONAR_NUM 2 //초음파 센서 수
#define MAX_DISTANCE 100 //cm단위 최대 값 (초음파 최대값)
#include <NewPing.h> //초음파 관련 헤더파일임! (설치 필요)
float UltrasonicSensorData[SONAR_NUM]; //센서 데이터 1차배열 생성(실수)

void read_ultrasonic_sensor(void) //초음파 값 읽어들이는 함수 
{
    NewPing sonar[SONAR_NUM] = {   // 초음파센서 배열 생성 
        NewPing(46, 47, MAX_DISTANCE),  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정) 앞
        NewPing(52, 53, MAX_DISTANCE),  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정) 오른쪽
    };
    UltrasonicSensorData[0] = sonar[0].ping_cm();
    UltrasonicSensorData[1] = sonar[1].ping_cm();
    
    if UltrasonicSensorData[0] == 0 {
        UltrasonicSensorData[0] = MAX_DISTANCE;
    }
    if UltrasonicSensorData[1] == 0 {
        UltrasonicSensorData[1] = MAX_DISTANCE;
    }

    if (UltrasonicSensorData[0] == MAX_DISTANCE) {
        object_exist = 0;
        Serial.println("초음파 - 전방 무한");
    }
    if(UltrasonicSensorData[0] != MAX_DISTANCE) {
        object_exist = 1;
        Serial.println("초음파 - 전방 오브젝트 확인 ");
    }
    Serial.println("front sonar");
    Serial.println(UltrasonicSensorData[0]);
    Serial.println("right sonar");
    Serial.println(UltrasonicSensorData[1]);
}

void No_Line_Sonar(void) { // 미로 속
    read_ultrasonic_sensor();
    int right_sonar_data = 300; //우측 초음파 센서 값 (일부러 150 이상인 300값을 줌(이상값)) [오차 감안해도, 수월한 전진을 위해 int형을 줌.]
    int want_distance = 50; // 미로 속에서 원하는 오른쪽부터의 유지 거리 값. (cm) -> 대회에서 연습 후 값 변경 필요
    while(right_sonar_data != MAX_DISTANCE){ // 오른쪽 초음파 센서 값이 무한대 (200)을 보일 때 까지 진행
        read_ultrasonic_sensor();
        right_sonar_data = UltrasonicSensorData[1];
        if (right_sonar_data > want_distance) { //무한대는 아닌데, 왼쪽으로 많이 가있음.
            steering_control(30); //오른쪽으로 턴해서 가까워짐
            motor_control(1, 70);
            delay(100);
        }
        else if (right_sonar_data < want_distance) { //오른쪽으로 많이 가있음.
            steering_control(-30); //왼쪽으로 턴해서 멀어짐
            motor_control(1, 70);
            delay(100);
        }
        else{ // 정상 주행 상태 (목표 주행 값 == 센서 값)
            steering_control(0);
            motor_control(1, 70); 
            delay(100);
        }
    }
    // 2차 우회전 필요.
}
void setup() {
    int loop_num = 1;
    for (int i = 0; i < NPIXELS; i++){
        LineSensor_Data_Adaption[i] = 0; //이미지를 나타내는 전체 픽셀들 보정 데이터를 0으로 초기화
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
// -------------------------------- 카메라/라인센싱 셋업 부분 끝 --------------------------------
// -------------------------------- 서보모터 셋업 부분 시작 --------------------------------
    Steeringservo.attach(RC_SERVO_PIN); //사용할 핀 먼저 선언
    Steeringservo.write(NEURAL_ANGLE); //선언한 핀에 기본값 지정(전방)
    steering_control(0); //전방을 바라보도록 설정.
// -------------------------------- 서보모터 셋업 부분 끝 --------------------------------
// -------------------------------- DC모터 셋업 부분 시작 --------------------------------
    pinMode(MOTOR_DIR, OUTPUT); //DC모터 방향 핀을 디지털 출력 핀으로 지정
    pinMode(MOTOR_PWM, OUTPUT); //DC모터 속도 핀을 디지털 출력 핀으로 지정
// -------------------------------- DC모터 셋업 부분 끝 --------------------------------
    Serial.begin(115200);
}

void loop() {
    motor_control(1,70);
    Two_Line(); // 라인트레이싱
    if right_turn == 0 {
        read_ultrasonic_sensor(); //초음파 확인
    }
    else {
        UltrasonicSensorData[0] = MAX_DISTANCE + 100;
        UltrasonicSensorData[1] = MAX_DISTANCE + 100;
    }
    if (UltrasonicSensorData[0] <= 30){
        read_ultrasonic_sensor();
        while(UltrasonicSensorData[1] >= 20) {
            read_ultrasonic_sensor();
            steering_control(1, 30);
            motor_control(1,70);
        }
        No_Line_Sonar;
        // 2차 우회전
        read_ultrasonic_sensor();
        while(UltrasonicSensorData[1] <= 20){
            read_ultrasonic_sensor();
            steering_control(1, 30);
            motor_control(1,70);
        }
        right_turn = 1; // 2차 턴 완료한 뒤에는 초음파 끔
    }
}
