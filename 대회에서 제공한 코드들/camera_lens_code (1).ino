#define AOpin  A0     // Analog output - yellow
#define SIpin  22     // Start Integration - orange
#define CLKpin 23    // Clock - red
// Vcc - brown
// GND - black

#define NPIXELS 128  // No. of pixels in array

byte Pixel[NPIXELS]; // Field for measured values <0-255>

int LineSensor_Data[NPIXELS];           // line sensor data(original)
int LineSensor_Data_Adaption[NPIXELS];  // line sensor data(modified)
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor
int flag_line_adapation;          // flag to check line sensor adpation

#define FASTADC 1
// defines for setting and clearing register bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup() {
  // put your setup code here, to run once:
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023; //0;
    MIN_LineSensor_Data[i] = 0; //1023;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode (AOpin, INPUT);

  digitalWrite(SIpin, LOW);   // IDLE state
  digitalWrite(CLKpin, LOW);  // IDLE state

#if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;

  Serial.begin(115200);

}
void line_adaptation(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i])  MAX_LineSensor_Data[i] = LineSensor_Data[i];
    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i])  MIN_LineSensor_Data[i] = LineSensor_Data[i];
  }

  /*for (i = 0; i < NPIXELS; i++)
    {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print("   : ");
    Serial.print(MAX_LineSensor_Data[i]);
    Serial.print(" | ");
    Serial.print(MIN_LineSensor_Data[i]);
    Serial.println(" ");
    }*/
}
void read_line_sensor(void)
{
  int i;

  delayMicroseconds (1);  /* Integration time in microseconds */
  delay(10);              /* Integration time in miliseconds  */


  digitalWrite (CLKpin, LOW);
  digitalWrite (SIpin, HIGH);
  digitalWrite (CLKpin, HIGH);
  digitalWrite (SIpin, LOW);

  delayMicroseconds (1);

  for (i = 0; i < NPIXELS; i++) {
    Pixel[i] = analogRead (AOpin) / 4 ; // 8-bit is enough
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
  }

  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data[i], MAX_LineSensor_Data[i], 0, 256);
  }

}
void loop() {
  // put your main code here, to run repeatedly:

  int i;

  read_line_sensor();

  if (digitalRead(CLKpin) == HIGH)
  {
  //  line_adaptation();
    flag_line_adapation = 1;
  }

  for (i = 0; i < NPIXELS; i++)
  {
    if (digitalRead(CLKpin) == LOW)    Serial.print(LineSensor_Data_Adaption[i]); // Serial.print(LineSensor_Data[i] );
    else                               Serial.print ((byte)Pixel[i] + 1);
    // Serial.print ((byte)Pixel[i] + 1);
    Serial.print(" ");
  }

  Serial.println("  ");
  delay(100);
}
