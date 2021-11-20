import processing.serial.*; 

int i; 
Serial myPort; 
float data;
float[] line_data = new float[128];

public static int ON = 1;
public static int OFF = 0; 

void setup() 
{ 

  String portName ;
  size (1000, 1000); 
  delay(1000);  

  println(Serial.list()); 
  portName= Serial.list()[0];


  myPort = new Serial(this, "/dev/cu.usbserial-1130", 115200); //실제 값을 받은 아두이노와 연결된 포트를 설정합니다.
}

void draw() 
{ 
  // data = 10;
  background(255); //배경을 흰색으로 설정합니다.

  int i;
  for (i=0; i<128; i++)
  {
    rect(20+5*i, 950, 5, -line_data[i]); //원을 그리는데 원의 중심은 (500,400)에 있으며 
    fill(0, 0, 255);
  }
} //변수를 사용합니다. 원 안은 검은색으로 칠합니다.

/*
void serialEvent(Serial p) 
 { 
 
 String message = myPort.readStringUntil(10); //시리얼 포트에 있는 데이터 값은 엔터가 입력될 때 
 if (message != null) //까지 받습니다. 만약 데이터가 있으면 
 { 
 // print("Received: "); println(message); //출력합니다. 
 data = float(message); //그리고 data변수에 저장합니다. 
 } 
 
 }
 */

void serialEvent(Serial port) //Reading the datas by Processing.
{
  int i, data_temp;
  String input = port.readStringUntil('\n');
  if (input != null)
  {
    input = trim(input);
    String[] values = split(input, " ");
    if (values.length == 128)
    {

      for (i=0; i<128; i++)
      {
        data_temp  =  int(values[i]);

        line_data[i] = map(data_temp, 0, 256, 0, 800);
        print(line_data[i]);
        print(" ");
      }
      println(" ");
    }
  }
}
