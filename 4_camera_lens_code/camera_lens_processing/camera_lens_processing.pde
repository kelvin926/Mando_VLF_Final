// 프로세싱을 하기 전에 아두이노 업로드를 먼저 해야함.

import processing.serial.*; 

int i; 
Serial myPort; 
float data;
float[] line_data = new float[128]; // 동적 할당

public static int ON = 1; // 정적, 공개형(cpp)
public static int OFF = 0; 

void setup() 
{ 

  String portName;
  size (1000, 1000); // 프로세싱으로 그릴 전체 사이즈를 지정 (1000x1000)
  delay(1000);  

  println(Serial.list()); // 연결된 시리얼 번호들 출력
  portName= Serial.list()[0]; //잡히는 첫번째 포트를 포트 이름으로 함.


  myPort = new Serial(this, "/dev/cu.usbmodem101", 115200); //맥북은 위에 포트네임 코드로 작동을 안해서, 직접 입력했음.
  //myPort = new Serial(this, portName, 115200);
}

void draw() 
{ 
  // data = 10;
  background(255); //배경을 흰색으로 설정

  int i;
  for (i=0; i<128; i++)
  {
    rect(20+5*i, 950, 5, -line_data[i]); // rect(x좌표,y좌표,너비,높이) (그래프를 그리니까!)
    fill(0, 0, 255); //rgb (파란색)
  }
}

void serialEvent(Serial port) // 해당 포트로 시리얼이 들어온다면
{
  int i, data_temp;
  String input = port.readStringUntil('\n'); //input에 줄넘김이 올 때까지 시리얼 데이터를 쭉 저장
  if (input != null) // 뭔가가 들어온다면
  {
    input = trim(input); // trim:문자열의 시작과 끝에 있는 공백을 제거함.
    String[] values = split(input, " "); // input에 있는 데이터들을 공백마다 분리해서 string배열에 저장
    
    if (values.length == 128) //데이터가 가득 찼다면.
    {
      for (i=0; i<128; i++)
      {
        data_temp  =  int(values[i]); //배열 안에 있는 데이터 하나씩을 뽑아냄.
        line_data[i] = map(data_temp, 0, 256, 0, 800); // 데이터(최소:0, 최대:256)를 최소0, 최대800 범위로 변환하여 저장함.
        print(line_data[i]); //출력을 합시다람쥐
        print(" ");
      }
      println(" ");
    }
  }
}
