 #include <Servo.h> //Servo라이브러리 아두이노 프로그램에 설치해야합니당 아마 기본으로 깔려있을껄..?
#define RC_SERVO_PIN 40 //8번핀 할당
#define NEURAL_ANGLE 90 //기본 앵글: 90도 -> 전방 방향
Servo Steeringservo;
// ---------------카메라 초기 설정 시작---------------
#define AOpin  A0 // Analog output - 카메라 데이터 통신
#define SIpin  11 // Start Integration 디지털 11핀 - 카메라 데이터 전송을 알리는 핀
#define CLKpin 12  // Clock 디지털 12핀 - 데이터 전송을 위한 속도(클럭) 펄스를 주는 핀
#define NPIXELS 128 // 해상도
byte Pixel[NPIXELS]; // 얼마나 측정할 것인지. <0-255> (블랙 0, 흰색 255)
int LineSensor_Data_Adaption[NPIXELS]; // 보정된 라인센서 데이터 배열
int MAX_LineSensor_Data = 0; // 센서의 최대값 정의 (임의값)
int MIN_LineSensor_Data = 99999; // 센서의 최소값 정의 (임의값)
#define FASTADC 1 // 데이터 가속화
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
void read_ultrasonic_sensor(void);
int Line_Exist = 1; // 라인 존재 여부 (기본값 = 1)
int object_exist = 0; // 앞에 장애물 존재 여부
int right_turn = 0;
// ------------------ 카메라 초기 설정 끝 --------------------------------

#define SONAR_NUM 2 //초음파 센서 수
#define MAX_DISTANCE 200 //cm단위 최대 값 (초음파 최대값)

// ------------------ 라인 센싱 초기 설정 시작 --------------------------------
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
    
    line_adaptation();

    for (i = 0; i < NPIXELS; i++) // 0 ~ 1023까지의 데이터를 2번 과정을 거친 뒤에 -> 0 ~ 256
    {
        LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data, MAX_LineSensor_Data, 0, 256);
    }
}
// ------------------ 라인 센싱 초기 설정 끝 --------------------------------



// ------------------ DC모터 초기 설정 시작 --------------------------------
#define MOTOR_DIR 4 //노란친구(Encoder A) - D2(INT4) (방향 제어 - direction)
#define MOTOR_PWM 5 //흰색친구(Encoder B) - D3(INT5) (속도 제어 - speed (pwm))

void motor_control(int direction, int speed) //dc모터 컨트롤 함수 생성 
{
    digitalWrite(MOTOR_DIR, direction);   //방향 값을 디지털 값으로 모터에 입력
    analogWrite(MOTOR_PWM, speed); //속도 값을 아날로그 값으로 모터의 입력
}
// ------------------ DC모터 초기 설정 끝 --------------------------------

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

// -------------------------------- 이진화 시스템 시작 --------------------------------
#define threshold_value 15
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
// -------------------------------- 이진화 시스템 끝 --------------------------------


// 루프에 while문을 돌려서 안전수(두 라인 검출 시스템 on/off)를 확보하고, 라인 검출에 실패했을 때 미로 탐색 함수 작동, 한 라인 검출 실패 -> 한줄모드

// ------------------ 적응형 라인 검출 시스템 시작 --------------------------------
void Two_Line(void)
{
    read_line_sensor(); //라인 센싱부 작동~ -> 보정한 데이터 얻음.
    threshold(); // 이진화 함수
    int i;
    int left, right; //좌,우 라인 내부 값
    int center;
    int two_steer_data = 0;
    int sum = 0;
    for (i = 0; i < NPIXELS; i++) { // 왼쪽부터 순차적으로 올라갈 때 처음으로 라인이 존재하는 구간 검출
        if (LineSensor_Data_Adaption[i] == 255) {
            left = i;
            break;
        }
    }
    for (i = (NPIXELS-1); i >= 0; i--) { // 오른쪽부터 순차적으로 내려올 때 처음으로 라인이 존재하는 구간 검출
        if (LineSensor_Data_Adaption[i] == 255) {
            right = i;
            break;
        }
    }
    for (i = 0; i < NPIXELS; i++) {
        sum += LineSensor_Data_Adaption[i];
    }

    if(sum == 0) { // 라인 없음
        Line_Exist = 0;
    }

    else {
        Serial.print("left line ");
        Serial.println(left);
        Serial.print("right line ");
        Serial.println(right);
        center = ((left + right)/2);
        two_steer_data = center - 64;
        int alpha = 15;               // 라인 한줄의 왼쪽 오른쪽 폭 길이
        int beta = 70;               // 라인과 라인 사이의 검은 공간의 폭 길이
        int point = ((right - left) * 100)/128;
        if (right - left <= alpha) {          // 15는 어림잡아서 라인 한줄의 왼쪽 오른쪽 폭 길이이다
            if (point > 50) {
                if (center * 100 /128 >= 25) {
                    two_steer_data = center - 64 + (alpha/2) + (beta/2);
                }

                if (center * 100 /128 < 25) {
                    two_steer_data = center - 64 - (alpha/2) - (beta/2);
                }
            }

            if (point < 50) {
                if (center * 100 /128 <= 75) {
                    two_steer_data = center - 64 + (alpha/2) + (beta/2);
                }

                if (center * 100 /128 > 75) {
                    two_steer_data = center - 64 - (alpha/2) - (beta/2);
                }
            }
        }
        Serial.print("steer num");
        Serial.println(two_steer_data);
        steering_control(two_steer_data*2.7); // 가중치
    }
}
// ------------------ 적응형 라인 검출 시스템 끝 ------------------------------



// -------------------------------- 초음파 시스템 시작 --------------------------------
#include <NewPing.h> //초음파 관련 헤더파일임! (설치 필요)
float UltrasonicSensorData[SONAR_NUM]; //센서 데이터 1차배열 생성(실수)

NewPing sonar[SONAR_NUM] = {   // 초음파센서 배열 생성 
    NewPing(46, 47, MAX_DISTANCE),  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정) 앞
    NewPing(52, 53, MAX_DISTANCE),  // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정) 오른쪽
};

void read_ultrasonic_sensor(void) //초음파 값 읽어들이는 함수 
{
    UltrasonicSensorData[0] = sonar[0].ping_cm();
    UltrasonicSensorData[1] = sonar[1].ping_cm();
    // if(UltrasonicSensorData[0] == 0.0) //0.0을 넣었을 경우(완전 붙었다기보단 너무 멀어져서 값이 0이 된 것.)
    // {
    //     UltrasonicSensorData[0] = (float)MAX_DISTANCE; //측정값이 매우 커서 0이 나왔을 경우, 넣을 수 있는 최대값을 넣는다.
    // }
    // if(UltrasonicSensorData[1] == 0.0) //0.0을 넣었을 경우(완전 붙었다기보단 너무 멀어져서 값이 0이 된 것.)
    // {
    //     UltrasonicSensorData[1] = (float)MAX_DISTANCE; //측정값이 매우 커서 0이 나왔을 경우, 넣을 수 있는 최대값을 넣는다.
    // }
    
    if (UltrasonicSensorData[0] == 200) {
        object_exist = 0;
        Serial.println("전방 오브젝트 있음");
    }
    if(UltrasonicSensorData[0] != 200) {
        object_exist = 1;
        Serial.println("전방 오브젝트 없음");
    }
    Serial.println("front sonar");
    Serial.println(UltrasonicSensorData[0]);
    Serial.println("right sonar");
    Serial.println(UltrasonicSensorData[1]);
}
// -------------------------------- 초음파 시스템 끝 --------------------------------



// ------------------ 라인 없을 때 시스템 시작 -----------------------------
void No_Line_Turn(void){
    int front_sensor = 300, right_sensor = 300;
    while (front_sensor >= 50) {  // 50cm 앞까지 벽이 오도록 전방으로 감. - 대회에서 연습 후 값 변경 필요
        Serial.println("전방 센서 감지 중");
        steering_control(0);
        read_ultrasonic_sensor();
        motor_control(1, 70);
        front_sensor = UltrasonicSensorData[0]; // 0 : 전방 센서 , 2 : 우측 센서
    }
// 이동 완료
    while (right_sensor >= 30) { // 옆 센서가 30cm값을 가질 때 까지 우회전 - 대회에서 연습 후 값 변경 필요
        Serial.println("1차 노라인 우회전 중");
        steering_control(30);
        motor_control(1, 70);
        read_ultrasonic_sensor();
        right_sensor = UltrasonicSensorData[1];
    }
    steering_control(0);
    delay(300); // 0.3초 휴식
}

void No_Line_Sonar(void) { // 미로 속
    int right_sonar_data = 300; //우측 초음파 센서 값 (일부러 150 이상인 300값을 줌(이상값)) [오차 감안해도, 수월한 전진을 위해 int형을 줌.]
    int want_distance = 50; // 미로 속에서 원하는 오른쪽부터의 유지 거리 값. (cm) -> 대회에서 연습 후 값 변경 필요
    while(right_sonar_data != MAX_DISTANCE){ // 오른쪽 초음파 센서 값이 무한대 (200)을 보일 때 까지 진행
        read_ultrasonic_sensor();
        right_sonar_data = UltrasonicSensorData[1];
        if ((right_sonar_data < 200) && (right_sonar_data > want_distance)) { //무한대는 아닌데, 왼쪽으로 많이 가있음.
            steering_control(20); //오른쪽으로 턴해서 가까워짐
            motor_control(1, 70);
        }
        else if (right_sonar_data < want_distance) { //오른쪽으로 많이 가있음.
            steering_control(-20); //왼쪽으로 턴해서 멀어짐
            motor_control(1, 70);
        }
        else{ // 정상 주행 상태 (목표 주행 값 == 센서 값)
            steering_control(0);
            motor_control(1, 70); 
        }
    }
    Serial.println("2차 우회전 순간");
    // 오른쪽 초음파 센서 값이 무한대를 보였음. 2번째 우회전
    read_ultrasonic_sensor();
    while (UltrasonicSensorData[1] >= 50) //50cm까지 우회전
        read_ultrasonic_sensor();
        Serial.println("2차 노라인 우회전 중");
        steering_control(30);
        motor_control(1, 70);
    Two_Line(); //다시 라인 트레이싱
}

// ------------------ 라인 없을 때 시스템 끝 ------------------------------



// -------------------------------- 셋업 START --------------------------------
void setup() {

    // -------------------------------- 카메라/라인센싱 셋업 부분 시작 ------------------------------
    for (int i = 0; i < NPIXELS; i++) // NPIXELS = 128임.
    {
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
    Serial.begin(115200); // 115200속도로 시리얼 전송~

// 대회에서 앞에 장애물이 사라질 때 출발하는 시스템 시작 --------------------------------
    while (1) {
        read_ultrasonic_sensor();
    if (object_exist == 1) { // 앞에 장애물이 없음
        break; // 시작!
    }
}
// 대회에서 앞에 장애물이 사라질 때 출발하는 시스템 끝 --------------------------------
} 
// -------------------------------- 셋업 END --------------------------------


// -------------------------------- 서보모터 초기 설정 시작 --------------------------------


void steering_control(int steer_angle) //앞바퀴 스티어링 함수.
{
    Steeringservo.write(NEURAL_ANGLE + steer_angle);  // 기본값(90도)에 앵글 값 대입하여 스티어링 작동
}   
// -------------------------------- 서보모터 초기 설정 끝 --------------------------------


// -------------------------------- 루프 START --------------------------------
void loop() {
    if (Line_Exist == 1) {
        read_ultrasonic_sensor();
        motor_control(1, 70); //앞 방향으로 y의 속도만큼
        Two_Line(); // 라인 센싱 함수
    }
    else { //라인 감지 실패 - Line_Exist == 0
        read_ultrasonic_sensor();
        if (object_exist == 0){ //오브젝트 감지 여부 -> 없음
            read_ultrasonic_sensor();
            steering_control(0);
            motor_control(1, 70);
            delay(300);
            read_ultrasonic_sensor();
            Two_Line();
        }
        else { // 오브젝트 감지 결과 있음 -> 미로
            No_Line_Turn();
            No_Line_Sonar();
        }
    }
}
// -------------------------------- 루프 END --------------------------------
