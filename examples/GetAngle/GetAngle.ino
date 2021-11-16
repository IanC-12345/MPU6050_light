#include <MPU6050_light.h>

#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>

Servo servo4;
Servo servo8;


int period = 10;

unsigned long timer = 0;
unsigned long time;

double error_x;
double error_x_previous;
double x_setpoint = 98;

float error_y;
float error_y_previous;
float y_setpoint = 92.5;

float kp_x=1; 
float ki_x=0 ; 
float kd_x=1; 

float kp_y=1 ; 
float ki_y=0; 
float kd_y=1; 


double PID_p_x, PID_i_x, PID_d_x, PID_total_x;
double PID_p_y, PID_i_y, PID_d_y, PID_total_y;

MPU6050 mpu(Wire);


void setup() {
  Serial.begin(115200);
  Wire.begin();

  servo4.attach(4);
  servo8.attach(8);
  servo4.write(98);
  servo8.write(92.5);
  delay(500);
  Serial.println("Motor Finish Setup");
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
   delay(1000);
  mpu.calcOffsets();
  mpu.getFilterGyroCoef();
  mpu.update(); 
  Serial.println(-mpu.getAngleY());
  Serial.println("Done!\n");
}

void loop() {

   
    if(time<5000){
    time = millis();
    mpu.update();
    Serial.println("Please Wait for a while");
    Serial.println(time);
    }
    
    else{
      mpu.update(); 
      if((millis()-timer)>10){
         

  error_x = x_setpoint - mpu.getAngleX();
  error_y = y_setpoint - mpu.getAngleY();

    PID_p_y = kp_y * error_y;
    float diff_y = error_y - error_y_previous;     
    PID_d_y = kd_y*((error_y - error_y_previous)/period);    
    if(89.5 < error_y && error_y < 93.5)
    {
      PID_i_y = PID_i_y + (ki_y * error_y);
    }
    else
    {
      PID_i_y = 0;
    }
    
    PID_total_y = PID_p_y + PID_i_y + PID_d_y; 

     
    PID_p_x = kp_x * error_x;
    float diff_x = error_x - error_x_previous;     
    PID_d_x = kd_x*((error_x - error_x_previous)/period);
   if(95 < error_x && error_x < 101)
    {
      PID_i_x = PID_i_x + (ki_x * error_x);
    }
    else
    {
      PID_i_x = 0;  
    PID_total_x = PID_p_x + PID_i_x + PID_d_x;
    }

  
  servo4.write(PID_total_x);
  servo8.write(PID_total_y);
  Serial.println(-mpu.getAngleY());
  Serial.println(mpu.getAngleX());

  error_y_previous - error_y;  
  error_x_previous - error_x; 
  
  timer = millis();  

      }
    }
}
    

   


 
