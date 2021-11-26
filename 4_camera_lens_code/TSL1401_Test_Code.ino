/*
TSL1401test --- Taos TSL1401 image sensor chip 2010-07-24
datasheet: http://www.ams.com/eng/content/download/250163/975677/file/TSL1401CL.pdf

trace: http://ap.urpi.fei.stuba.sk/sensorwiki/index.php/TSL1401_Line_Sensor
other inos:
    - http://forums.parallax.com/showthread.php/125594-TSL1401-and-Arduino
    - https://github.com/ap-tech/Getting-started-with-the-TSL1401CL-linescan-camera-with-arduino-and-processing.-/blob/master/TSL1401CL%20linescan%20camera%20code./Linescane_camera_code/Linescane_camera_code.ino
    

*/


                    // Sensor interface: 
#define AOpin  0     // Analog output - yellow
#define SIpin  3     // Start Integration - orange
#define CLKpin 2     // Clock - red
                    // Vcc - brown
                    // GND - black

#define NPIXELS 128  // No. of pixels in array

byte Pixel[NPIXELS]; // Field for measured values <0-255>


#define FASTADC 1   
// defines for setting and clearing register bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))



void setup(void)
{
pinMode(SIpin, OUTPUT);
pinMode(CLKpin, OUTPUT);
//pinMode (AOpin, INPUT);

digitalWrite(SIpin, LOW);   // IDLE state
digitalWrite(CLKpin, LOW);  // IDLE state

#if FASTADC
// set prescale to 16
sbi(ADCSRA,ADPS2);
cbi(ADCSRA,ADPS1);
cbi(ADCSRA,ADPS0);
#endif

Serial.begin (115200);
}



void loop (void)
{
int i;
int expTime;


delayMicroseconds (1);  /* Integration time in microseconds */
delay(10);              /* Integration time in miliseconds  */


digitalWrite (CLKpin, LOW);
digitalWrite (SIpin, HIGH);
digitalWrite (CLKpin, HIGH);
digitalWrite (SIpin, LOW);

delayMicroseconds (1);            

/* and now read the real image */

for (i = 0; i < NPIXELS; i++) {
    Pixel[i] = analogRead (AOpin)/4 ; // 8-bit is enough
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
}


Serial.write ((byte)0);            // sync byte = 0
for (i = 0; i < NPIXELS; i++) {
    Serial.write ((byte)Pixel[i]+1);  

}
}