#include <Wire.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

const unsigned char OSS = 0;  // Oversampling Setting

float raw;
float raw0;

//tables generated from Excel, based on ratio of lifetime to BJ period of 3.5
//hence for 3 Hz peak freq, period T = 333 ms and lifetime is 3.5*333 ms = 1.17 s is how long it takes to drop to 1/e. 
//But with 80 points or 10 periods we actually get to a dropoff to about 5% of original value.
//To change *ratio* of BJ freq to lifetime we have to change these tables, but BJ freq can be adjusted manually with "deltat" variable below in setup

//revised to have sin/cos be purex  and one period

// (C) Orgasmatronics, Inc. 2013

//P = Cos[omega*t]
float P[] = {1.00,0.92,0.71,0.38,0.00,-0.38,-0.71,-0.92,-1.00,-0.92,-0.71,-0.38,0.00,0.38,0.71,0.92};
//Q = Sin[omega*t]*Exp[t/tau]
float Q[] = {0.00,0.38,0.71,0.92,1.00,0.92,0.71,0.38,0.00,-0.38,-0.71,-0.92,-1.00,-0.92,-0.71,-0.38};
      
float p=0.0;  //integrated sin power
float q=0.0;  //integrated cos power
//x = input variable array
float x[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float xmean = 0.0;
int sensorPin = A0;  // select the input pin Suctus
int m=0;
float F = 0.0;
float Fmax = 1023.0;
float gamma = 7.0;
float vold = 0.0;
float vnew = 0.0;

int deltat = 42;  //set BJ peak freq here, this 42 ms is about 3 Hz freq, deltat is period/8, 333ms/8 \approx 42 ms

//int deltat = 25;  //set BJ peak freq here, this 42 ms is about 3 Hz freq, deltat is period/8, 333ms/8 \approx 42 ms

float v=0.0;
float vmax = 255.0;

float k = 0.0;
float n = 0.0;
int tau = 300;

void setup()
{
  pinMode(6, OUTPUT);  //vibrator control pin, to HackOff
  pinMode(5, OUTPUT);
  pinMode(10, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  raw0 = bmp085ReadUP();
  
   pinMode(A0, INPUT);//suctus sensor in
   k = 0.9;//.3
   tau = 500;
   n = tau/deltat;
   //raw = -(bmp085ReadUP() - raw0)/5;
   xmean = bmp085ReadUP();
   for (m = 0; m < 16;m++){
   x[m] = xmean;
   }
  
}


void loop()  { 
  
  
  for (m = 15; m > 0; m--){
   x[m]=x[m-1]; 
  }
 // x[0] = analogRead(sensorPin)-xmean;  
  x[0] = bmp085ReadUP();  
  p=0.0;
  q=0.0;
  xmean =0.0;
  for (m = 0; m < 16;m++){
     p = p + P[m]*x[m];
     q = q + Q[m]*x[m];
  }

F = k*sqrt(p*p + q*q)/16;

  if (F > Fmax)
  {
    F = Fmax;
  }  

  vnew = vold + ((F*gamma - vold)/n);
   if (vnew > vmax)
  {
    vnew = vmax;
  }  
 
  vold = vnew;
  analogWrite(6,vnew); 
  analogWrite(5,vnew); 
  analogWrite(10,vnew); 

//  analogWrite(6,255); 
  delay(deltat);
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
       ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}




