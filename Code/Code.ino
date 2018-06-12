#include "MPU9250.h"
#include "Kalman.h"

int16_t gyro_data[3]={0,0,0};
int16_t accel_data[3]={0,0,0};

MPU9250 mpu;

double ax,ay,az;
double gx,gy,gz;

double gyroAngle= 0.00;
double accelAngle;
double Angle;

uint32_t currentTime; 

Kalman kalmanAngle;

void setup() 
{
    mpu.initMPU9250();
    
    currentTime = micros();
    
    mpu.readGyroData(gyro_data);
    mpu.readAccelData(accel_data);
    
    ax=accel_data[0]; ay=accel_data[1]; az=accel_data[2];
    gx=gyro_data[0]; gy=gyro_data[1]; gz=gyro_data[2];

    accelAngle = atan(ay/(sqrt((ax*ax)+(az*az)))) * (180/3.14159);
  
    kalmanAngle.setAngle(accelAngle);

    gyroAngle = accelAngle;
}

void loop() 
{
    mpu.readGyroData(gyro_data);
    mpu.readAccelData(accel_data);
    
    ax=accel_data[0]; ay=accel_data[1]; az=accel_data[2];
    gx=gyro_data[0]; gy=gyro_data[1]; gz=gyro_data[2];

    double dt = ((double)(micros() - currentTime)) / 1000000.0; 
    currentTime = micros();
    
    double omegaX = gx / 131.0;

    gyroAngle = gyroAngle + (omegaX*dt);
    
    accelAngle = atan(ay/(sqrt((ax*ax)+(az*az)))) * (180/3.14159);

    if ((accelAngle < -90 && Angle > 90) || (accelAngle > 90 && Angle < -90))
    {
        kalmanAngle.setAngle(accelAngle);
        Angle = accelAngle;
        gyroAngle = accelAngle;
    } 
    else
    {
        Angle = kalmanAngle.getAngle(accelAngle, omegaX , dt);         // Calculate the angle using a Kalman filter
    }

    if(gyroAngle < -180 || gyroAngle > 180)
    {
        gyroAngle = Angle;
    }

    double RawRPM  = Pid(Angle-0.7);
    
    MotorController(RawRPM);
}

//------------------------------------------------------------MOTOR CONTROLLER------------------------------------------------------------

int left_forw = 12;
int left_back = 11;
int right_forw = 8;
int right_back = 7;

float left_pwm = 9;
float right_pwm = 6;

void MotorPin() 
{
  pinMode(left_forw,OUTPUT);
  pinMode(left_back,OUTPUT);
  pinMode(left_pwm,OUTPUT);
  
  pinMode(right_forw,OUTPUT);
  pinMode(right_back,OUTPUT);
  pinMode(right_pwm,OUTPUT);
}

void MotorController(double output) 
{
    if (output<0)
    {
       digitalWrite(left_forw,HIGH);
       digitalWrite(left_back,LOW);
       digitalWrite(right_forw,HIGH);
       digitalWrite(right_back,LOW);
    }
    else if(output>0)
    {
      digitalWrite(left_forw,LOW);
      digitalWrite(left_back,HIGH);
      digitalWrite(right_forw,LOW);
      digitalWrite(right_back,HIGH);
    }
    else
    {
      digitalWrite(left_forw,HIGH);
      digitalWrite(left_back,HIGH);
      digitalWrite(right_forw,HIGH);
      digitalWrite(right_back,HIGH);
    }
  
    int PWM = map(abs(output),0,80,150,255);        
  
    analogWrite(left_pwm,PWM);
    analogWrite(right_pwm,PWM);
}

//------------------------------------------------------------------ PID -------------------------------------------------------------------

float Kp=13;
float Ki=0;
float Kd=1;

float Setpoint_angle=1,Output;
float error,prev_error=0,s_error=0,d_error=0;

double Pid(double Input_angle) 
{    
      error = Input_angle - Setpoint_angle;
      if(abs(error)<.5)
      error=0;
      d_error = error - prev_error;
      s_error += error;
      prev_error = error;
      
      Output = (error * Kp) + (s_error * Ki) + (d_error * Kd);
      
      int temp=80;  

      if(Output>temp)
      {
          Output=temp;
      }
      if(Output<-temp)
      {
        Output=-temp;
      }

      return Output;
}


