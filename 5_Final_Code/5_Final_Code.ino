// ---------------카메라 초기 설정 시작---------------
#define AOpin  A0 // Analog output - 카메라 데이터 통신
#define SIpin  11 // Start Integration 디지털 11핀 - 데이터 전송을 알리는 핀
#define CLKpin 12  // Clock 디지털 12핀 - 데이터 전송을 위한 속도(클럭) 펄스를 주는 핀
#define NPIXELS 128 // 한 배열당 몇 픽셀을 넣을 것인지 정의 
byte Pixel[NPIXELS]; // 얼마나 측정할 것인지. <0-255> (블랙 0, 흰색 255)
int LineSensor_Data[NPIXELS]; // 원본 라인센서 데이터 배열
int LineSensor_Data_Adaption[NPIXELS]; // 보정된 라인센서 데이터 배열
int MAX_LineSensor_Data[NPIXELS]; // 센서의 최대값 정의
int MIN_LineSensor_Data[NPIXELS]; // 센서의 최소값 정의
int flag_line_adapation; // 라인센서 확인을 위한 플래그 변수 (사실 뭔말인지 모르겠음,,)
                        // 깃헙에 찾아봐도 관련 레퍼런스도 없고,,, 참 그러네 ㅜ
#define FASTADC 1 //아날로그 리딩을 가속화 시키는 FASTADC 변수 정의 (카메라 값을 빠르게 받아오기 위함.)
                    // 레지스터 비트 설정 및 소거에 대한 정의 [가속화 관련]
                    // http://electronics.arunsworld.com/basic-avr/ -> 관련 레퍼런스 첨부합니당
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) //어샘블리 쪽 언어같은데,, 이 문장 아직도 사용하는거 처음봄,,
                                                    // cbi(sfr, bit) : Clear bit bit in register sfr.
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
                                                    // sbi(sfr, bit) : Set bit bit in register sfr.
                                                    // 아마 카메라에서 데이터를 가지고 올 때, 임시 레지스터를 말하는 것 같은데,,, 이해하기가 초큼 어렵네요..
                                                    // 카메라에서 데이터 가지고 올 때 가속화시킬려면 해당 구문이 꼭 필요하다고 합니당,,, 이해는 안하는게 정신건강에 좋을듯.
// ------------------ 카메라 초기 설정 끝 --------------------------------



// ------------------ 라인 센싱 초기 설정 시작 --------------------------------
void line_adaptation(void) // 라인 보정부
{
    int i;
    for (i = 0; i < NPIXELS; i++)
    {
        if (LineSensor_Data[i] >= MAX_LineSensor_Data[i])  MAX_LineSensor_Data[i] = LineSensor_Data[i]; //센서 데이터가 이전의 최대 데이터보다 클 경우, 최대 데이터를 센서 데이터로 입력
        if (LineSensor_Data[i] <= MIN_LineSensor_Data[i])  MIN_LineSensor_Data[i] = LineSensor_Data[i]; //센서 데이터가 이전의 최소 데이터보다 작을 경우, 최소 데이터를 센서 데이터로 입력
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

    for (i = 0; i < NPIXELS; i++)
    {
        Pixel[i] = analogRead (AOpin) / 4 ; // 8-bit is enough - 픽셀배열에 각 이미지 픽셀 값을 넣음
        digitalWrite (CLKpin, LOW); // CLK핀 대기상태 - 데이터 동기화 중지 (위 코드에서 High인 상태에서 해당 반복문이 시작되기에, 첫 픽셀은 기본으로 들어감.)
        delayMicroseconds (1); // 1ms대기
        digitalWrite (CLKpin, HIGH); //CLK핀 작동상태 - 데이터 동기화 시작 (이게, 데이터가 많으니까 한 픽셀에 들어갈 양만 clk값에 따라 딱 넣고 멈췄다가 다음 픽셀에 넣고 멈췄다가 이런 느낌인가봐.)
    }

    for (i = 0; i < NPIXELS; i++)
    {
        LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data[i], MAX_LineSensor_Data[i], 0, 256); //map(변환할 수, 현재 범위의 최소값, 현재 범위의 최대값, 목표 범위의 최소값, 목표 범위의 최대값)
        /* 보정데이터에 각 픽셀마다의 값을 가져와서 0~255값을 넣어서 최종적으로 0~255의 값을 갖게 함.
        // 한마디로, 카메라로 이미지를 찍었으면, 가장 흰색에 가까운 값을 최대로 두고, 가장 검은색에 가까운 값을 최소로 둠.
        // 이후 최대값을 255로 기준을 삼고, 최소값을 0으로 두어서, 상황에 따라 능동적으로 색을 분리할 수 있게함.
        // 검은색에 최대한 가까운 값을 0으로 보정, 흰색에 가장 가까운 값을 255로 고정해서 범위를 능동적으로 조절한다.
        대충 느낌 알꺼라 믿으 */
    }
}
// ------------------ 라인 센싱 초기 설정 끝 --------------------------------



// ------------------ DC모터 초기 설정 시작 --------------------------------
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
// ------------------ DC모터 초기 설정 끝 --------------------------------



// -------------------------------- 서보모터 초기 설정 시작 --------------------------------
#include <Servo.h> //Servo라이브러리 아두이노 프로그램에 설치해야합니당 아마 기본으로 깔려있을껄..?
#define RC_SERVO_PIN 8 //8번핀 할당
#define NEURAL_ANGLE 90 //기본 앵글: 90도 -> 전방 방향
#define LEFT_STEER_ANGLE -30 //좌측 스티어링 각도 지정
#define RIGHT_STEER_ANGLE 30 //우측 스티어링 각도 지정
Servo Steeringservo; //서보를 사용하는 함수를 미리 선언
int Steering_Angle = NEURAL_ANGLE;//기본 스티어링 값 기본값으로 지정(전방)

void steering_control(int steer_angle) //앞바퀴 스티어링 함수.
{
    if(steer_angle>=RIGHT_STEER_ANGLE) steer_angle = RIGHT_STEER_ANGLE; //스티어링 앵글 값이 오른쪽 스티어링 값 이상일 경우, 앵글에 오른쪽 스티어링 값 넣음 
    if(steer_angle<=LEFT_STEER_ANGLE)  steer_angle = LEFT_STEER_ANGLE; //이 코드도 위랑 비슷하게 작동
    Steeringservo.write(NEURAL_ANGLE + steer_angle);  // 기본값(90도)에 앵글 값 대입하여 스티어링 작동 
}
// -------------------------------- 서보모터 초기 설정 끝 --------------------------------



// -------------------------------- 이진화 시스템 시작 --------------------------------
#define threshold_value 180
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



// -------------------------------- 센터링(무게중심) 시스템 시작 --------------------------------
#define camera_pixel_offset 0 //센터가 0이 출력되도록 보정해주는 값 (수정 필요)
void steering_by_camera(void)
{
    int i;
    long sum = 0;
    long x_xum = 0;
    int steer_data = 0;
    for (i = 0; i < NPIXELS; i++)
    {
        sum += LineSensor_Data_Adaption[i];
        x_xum += LineSensor_Data_Adaption[i] * i;
    }
    steer_data = (x_xum/sum) - NPIXELS/2 + camera_pixel_offset;

    steering_control(steer_data*2); // 곱하는 값은 가중치.

    Serial.println(steer_data);
}
// -------------------------------- (무게중심) 시스템 끝 --------------------------------



// -------------------------------- 셋업 START --------------------------------
void setup() {
    // -------------------------------- 카메라/라인센싱 셋업 부분 시작 ------------------------------
    int i;
    for (i = 0; i < NPIXELS; i++) // NPIXELS = 128임.
    {
        LineSensor_Data[i] = 0; // 이미지를 나타내는 전체 픽셀들 오리지널 데이터를 0으로 초기화
        LineSensor_Data_Adaption[i] = 0; //이미지를 나타내는 전체 픽셀들 보정 데이터를 0으로 초기화
        MAX_LineSensor_Data[i] = 1023; //최대값을 전부 1023으로 초기화
        MIN_LineSensor_Data[i] = 0; //최소값을 전부 0으로 초기화
    }
    pinMode(SIpin, OUTPUT); //카메라 SI(카메라 시작) 디지털 핀 출력모드
    pinMode(CLKpin, OUTPUT); //카메라 CLK(카메라 데이터 전송 주기 동기화) 디지털 핀 출력모드
    pinMode (AOpin, INPUT); //카메라 A0 아날로그 핀 입력모드
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
    motor_control(0, 50); //1의 방향으로 50의 속도만큼

    threshold(); // 이진화 함수
    steering_by_camera(); //센터링(무게중심) 함수
    for (i = 0; i < NPIXELS; i++) //i에 픽셀 배열 순서 값이 촥촥 들어가겠죵..?
    {

        // 이진수 데이터가 아닌 RAW데이터를 확인 할 때에는 이 코드들을 살리기! + 이진화 함수 내용 끄기
        /*
        if (digitalRead(CLKpin) == LOW) // 카메라 데이터 송수신 동기화 X (이미 끝난 상태)
            Serial.print(LineSensor_Data_Adaption[i]); // 보정된 데이터를 시리얼에 출력함
        else // 카메라 데이터 송수신 동기화 중임. 
            Serial.print ((byte)Pixel[i] + 1); 
                //다음 픽셀 값을 출력함. 위에 픽셀값 넣는 반목문을 보면, CLK핀이 High일 경우 바로 다음에 i값이 올라가서 다음 픽셀 값을 가지게 됨. 따라서 1을 더해 다음 픽셀 값을 출력하는 것.
                // CLK값이 LOW라는 것은 해당 픽셀 값에 넣고, 1ms간 대기 중이라는 것이기에, 그냥 보정값을 출력하는 것. 이해가 될지 모르겠돠,,
        */

        // 프로세싱으로 라인 검출 데이터를 확인해야한다면 해당 코드를 살리기!
        // Serial.print(LineSensor_Data_Adaption[i]);
        // Serial.print(" ");
    }

    Serial.println("  ");
    delay(100);
}
// -------------------------------- 루프 END --------------------------------
