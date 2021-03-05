#include <Wire.h>
#include <PWM.h>
#include <Adafruit_BMP085.h>


// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

#define compressor_pwm 11
#define LED_debug 13
#define MATLAB_MODE 1
float Kp,Kd,Ki;
long curr_pressure, old_pressure, error, integral,desired_pressure,tolerance;
unsigned int val,state,time_delay;
unsigned long dt,t2,t1 ;
float sum;
Adafruit_BMP085 bmp;


void setup() {
  //InitTimersSafe();
  //SetPinFrequency(compressor_pwm,2000);
  pinMode(compressor_pwm, OUTPUT);
  pinMode(LED_debug, OUTPUT);
  Serial.begin(9600);
  old_pressure = bmp.readPressure();
  desired_pressure = 93000;
  
 /* States are as follows
  * 0: Bump test for Matlab Interface
  * 1: ON-OFF Controller with dead-band
  * 2: P-Controller with ITAE
  * 3: PI-Controller with IMC
  * 4: PID-Controller with IMC
  */ 
  state = 4;
 
  //**************//
  while (!bmp.begin()) {
     Serial.println("Could not find a valid BMP085 sensor, check wiring!");
     delay(500);
 
  }
}
  
void loop() {
  switch (state) {
    case 0:
    // Step Response for the FOPDT model, Resultant parameters: Tp = 10.855, theta_p=2.93
      while(1){
         
         t1=micros();
         curr_pressure = bmp.readPressure();
         //analogWrite(compressor_pwm,255);
         pwmWrite(compressor_pwm,128);
         if(MATLAB_MODE){
          Serial.write(curr_pressure/256/256);
          Serial.write(curr_pressure/256);
          Serial.write(curr_pressure%256);
        }
        else{
          Serial.print(desired_pressure);
          Serial.print("\t");
          Serial.println(curr_pressure);
        }
         t2=micros();
         dt=t2-t1;
         delayMicroseconds(35000-dt);
      }
      break;
      case 1:// ON-OFF Controller with Dead band
      tolerance = 0;
      desired_pressure = 93000; 
      while(1){
        curr_pressure = bmp.readPressure();
        if(curr_pressure < desired_pressure-tolerance){
          digitalWrite(compressor_pwm,HIGH);
        }
        else if(curr_pressure >= desired_pressure+tolerance){
          digitalWrite(compressor_pwm,LOW);
        }
        if(MATLAB_MODE){
          Serial.write(curr_pressure/256/256);
          Serial.write(curr_pressure/256);
          Serial.write(curr_pressure%256);
        }
        else{
          Serial.print(desired_pressure);
          Serial.print("\t");
          Serial.println(curr_pressure);
        }
      }
      break;
    case 2: // Aggressive P-Controller with ITAE  
      Kp = 0.09425;
      Kd = 0;
      Ki = 0;
      desired_pressure = 93000; 
      while(1){
        t1=micros();
        curr_pressure = bmp.readPressure();
        error = desired_pressure - curr_pressure;
        sum += error*0.035;
        val = Kp*error - Kd*(curr_pressure - old_pressure)/0.035 + Ki*sum;
        val = constrain(val, 0, 255);
        t2=micros();
        if(MATLAB_MODE){
          Serial.write(curr_pressure/256/256);
          Serial.write(curr_pressure/256);
          Serial.write(curr_pressure%256);
        }
        else{
          Serial.print(desired_pressure);
          Serial.print("\t");
          Serial.println(curr_pressure);
        }
        dt=t2-t1;
        delayMicroseconds(35000-dt);
        old_pressure = curr_pressure;
        analogWrite(compressor_pwm,val);
      }
      break;
    case 3: // PI-Controller with IMC
      Kp = 0.0785;
      Kd = 0;
      Ki = 0.0072;
      desired_pressure = 93000; 
      while(1){
        t1=micros();
        curr_pressure = bmp.readPressure();
        error = desired_pressure - curr_pressure;
        sum += error*0.035;
        val = Kp*error - Kd*(curr_pressure - old_pressure)/0.035 + Ki*sum;
        val = constrain(val, 0, 255);
        t2=micros();
        if(MATLAB_MODE){
          Serial.write(curr_pressure/256/256);
          Serial.write(curr_pressure/256);
          Serial.write(curr_pressure%256);
        }
        else{
          Serial.print(desired_pressure);
          Serial.print("\t");
          Serial.println(curr_pressure);
        }
        dt=t2-t1;
        delayMicroseconds(35000-dt);
        old_pressure = curr_pressure;
        analogWrite(compressor_pwm,val);
      }
      break;
    case 4: // PID controller with IMC
      Kp = 0.1233;
      Kd = 0.1591;
      Ki = 0.01;
      desired_pressure = 93000; 
      while(1){
        t1=micros();
        curr_pressure = bmp.readPressure();
        error = desired_pressure - curr_pressure;
        sum += error*0.035;
        val = Kp*error - Kd*(curr_pressure - old_pressure)/0.035 + Ki*sum;
        val = constrain(val, 0, 255);
        t2=micros();
        if(MATLAB_MODE){
          Serial.write(curr_pressure/256/256);
          Serial.write(curr_pressure/256);
          Serial.write(curr_pressure%256);
        }
        else{
          Serial.print(desired_pressure);
          Serial.print("\t");
          Serial.println(curr_pressure);
        }
        dt=t2-t1;
        delayMicroseconds(35000-dt);
        old_pressure = curr_pressure;
        pwmWrite(compressor_pwm,val);
      }
    break;
  }

}
