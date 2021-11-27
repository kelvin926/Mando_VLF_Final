
#include <NewPing.h> //초음파 관련 헤더파일임!
#define SONAR_NUM 2 //초음파 센서 번호를 부여 (1번)
#define MAX_DISTANCE 150 //cm단위 최대 값 (초음파 최대값)
float UltrasonicSensorData[SONAR_NUM];//센서 데이터 1차배열 생성(실수)

NewPing sonar[SONAR_NUM] = {   // 초음파센서 배열 생성 
    NewPing(9, 10, MAX_DISTANCE),
    NewPing(3, 2, MAX_DISTANCE) // NewPing(Trig(송신)핀 번호, Echo(수신)핀 번호, 최대측정거리) [초음파] (개체 설정)
};

void serial_com(void)
{
    Serial.print("Sonar1 : ");
    Serial.println(UltrasonicSensorData[0]);
    Serial.print("Sonar2 : ");
    Serial.println(UltrasonicSensorData[1]);
}



void read_ultrasonic_sensor(void) //초음파 값 읽어들이는 함수 
{
    for(int i=0; i<=1; i++){
        UltrasonicSensorData[i]= sonar[i].ping_cm();
        if(UltrasonicSensorData[i] == 0.0) {    //0.0을 넣었을 경우(완전 붙었다기보단 너무 멀어져서 값이 0이 된 것.)
            UltrasonicSensorData[i] = (float)MAX_DISTANCE; //측정값이 매우 커서 0이 나왔을 경우, 넣을 수 있는 최대값을 넣는다.
            //  Serial.println(UltrasonicSensorData[i]); //DEBUG
        }
    }
}

void setup() {
    // put your setup code here, to run once:
     Serial.begin(115200); //통신 속도 115200
}

void loop() {
    // put your main code here, to run repeatedly:
    read_ultrasonic_sensor(); //초음파 센서 값 리딩 
    serial_com(); //시리얼에 계속 띄움.
}
