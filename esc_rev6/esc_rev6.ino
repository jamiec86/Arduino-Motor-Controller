#include <Wire.h>
#include <Servo.h>
#include "Kalman.h"

/*i2c addresses*/
#define gyroAddress 0x68
#define adxlAddress 0x53

/*Kalman filter handels*/
Kalman kalmanX;
Kalman kalmanY;


/*PID variables*/
unsigned long lastTime;
double Input, Output, Setpoint = 180;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
double error;

/*Kalman Filter parmaeters*/
double zeroValue[5] = { -200, 44, 660, 52.3, -18.5}; 

/*Motor handels*/
Servo myMotor1;
Servo myMotor2;
Servo myMotor3;
Servo myMotor4;


/* All the angles start at 180 degrees */
double gyroXangle = 180;
double gyroYangle = 180;

double compAngleX = 180;
double compAngleY = 180;
double xAngle;
double yAngle;

/*Will be removed once roll PID controller is created*/
double roll_error;

double pitch_controler;
double roll_controler;

double thrust;

 double initial_accXangle;
 double initial_accYangle;
 double initial_accZangle;
 unsigned long timer;

boolean armed=0;

/*values to be sent to the individual motors*/
int val1;
int val2;
int val3;
int val4;

/*Incoing serial character*/
char ch;

void setup()
{
  // Put the motor to Arduino pin #9
  myMotor1.attach(9);
  // Put the motor to Arduino pin #10
  myMotor2.attach(10);
  // Put the motor to Arduino pin #11
  myMotor3.attach(11);
  // Put the motor to Arduino pin #12
  myMotor4.attach(12);

  // Required for I/O from Serial monitor
  Serial.begin(9600);
  Wire.begin();
  Serial.println("initializing");
  i2cWrite(adxlAddress, 0x31, 0x09); // Full resolution mode
  i2cWrite(adxlAddress, 0x2D, 0x08); // Setup ADXL345 for constant measurement mode

  i2cWrite(gyroAddress, 0x16, 0x03); // this puts your gyro at +-2000deg/sec  and 98Hz Low pass filter
  i2cWrite(gyroAddress, 0x15, 0x09); // this sets your gyro at 100Hz sample rate

  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  timer = micros();
  initial_accXangle= getXangle();
  initial_accYangle= getYangle();

  
  // Print a startup message
  Serial.println("initializing");
  armed=0;
  thrust=0;
  val1=0;
  val2=0;
  val3=0;
  val4=0;
  myMotor1.write(val1);
  myMotor2.write(val2);
  myMotor3.write(val3);
  myMotor4.write(val4);
  /*set limits of the PID controller*/
  SetOutputLimits(-180, 180);
}


void loop(){
  Compute();
  /* set the PID parmeters (Kp, Ki, Kd)*/
  SetTunings(0.2,0.001,0.0001);
  
  double gyroXrate = -(((double)readGyroX() - zeroValue[3]) / 14.375);
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Without any filter

  double gyroYrate = (((double)readGyroY() - zeroValue[4]) / 14.375);
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000); // Without any filter

  double accXangle = getXangle();
  double accYangle = getYangle();

  compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle);
  compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

  xAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter
  yAngle = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter

  timer = micros();
  
  /*serial print for chart recording*/
   Serial.print(xAngle); //Serial.print("\t"); 
   Serial.print(",");
   Serial.print(pitch_controler);
   Serial.print(",");
   Serial.print(error);
   Serial.print("\n");
   /*end of chart record*/
   
   /*text output
    Serial.print("Ang x"); Serial.print("\t"); 
    Serial.print("Ang y"); Serial.print("\t"); 
    Serial.print("Ang x comp"); Serial.print("\t"); 
    Serial.print("Ang y comp"); Serial.print("\t");
    Serial.print("\n");
    Serial.print(xAngle); //Serial.print("\t"); 
    Serial.print(yAngle); //Serial.print("\t");
    Serial.print(xAngle-initial_accXangle); //Serial.print("\t");Serial.print("\t");
    Serial.print(yAngle-initial_accYangle);// Serial.print("\t");
    Serial.print("\n");
  Text Output */
  
 delay(200);
 
  Input = xAngle;
  roll_error = yAngle - 180;
 
  pitch_controler= Output;
  roll_controler=roll_error*0.2 + (roll_error * (double)(micros() - timer) / 1000000);
 
            if (armed==1) {
                val1 = thrust + pitch_controler;
               if (val1>180) val1=180;
               if (val1<0) val1=0;
               val2 = thrust + roll_controler;
               if (val2>180) val2=180;
               if (val2<0) val2=0;
               val3 = thrust - pitch_controler;
               if (val3>180) val3=180;
               if (val3<0) val3=0;
               val4 = thrust - roll_controler;
               if (val4>180) val4=180;
               if (val4<0) val4=0;
            }
 
  // If there is incoming value
  if(Serial.available() > 0)
  {
   
    // read the value
    
    ch = Serial.read();

    /* If ch isn't a newline (linefeed) character */
       if (ch != 10){ 
          switch (ch) {
             case 97: //a
              Serial.println("arm");
              val1=0;
              val2=0;
              val3=0;
              val4=0;
              armed=1;
            break;
            case 43: //+
              if (armed==1){
                thrust+=1;          
              }
            break;
            case 45: //-
              if (armed==1){
                thrust -=1;
              }
            break;
            case 48: //0
              if (armed==1){
               thrust = 5;
              }
            break;
            case 49: //1
              if (armed==1){
               thrust = 10;
              }
            break;
            case 50: //2
              if (armed==1){
               thrust = 20;
              }
            case 51: //3
              if (armed==1){
               thrust = 30;
              }
            break;
            case 52: //4
              if (armed==1){
                thrust = 40;
              }
            break;
            case 53: //5
              if (armed==1){
                thrust = 50;
              }
            break;
            case 54: //6
              if (armed==1){
               thrust = 60;
              }
            break;
            case 55: //7
              if (armed==1){
               thrust = 70;
              }
            break;
            case 93: //]
             Serial.println("Shutdown");
             thrust = 0;
             val1=0;
              val2=0;
              val3=0;
              val4=0;
             myMotor1.write(0);
              myMotor2.write(0);
              myMotor3.write(0);
              myMotor4.write(0);
              armed=0;
            break;
             case 105: //i
             Serial.println("Shutdown");
             thrust = 0;
             val1=0;
              val2=0;
              val3=0;
              val4=0;
              myMotor1.write(0);
              myMotor2.write(0);
              myMotor3.write(0);
              myMotor4.write(0);
              armed=0;
            break; 
          
           case 114:Serial.println("Pitch Forward");
            if (armed==1){
              val1-=1;
              val3+=1;
            }
            break;
            case 99:Serial.println("Pitch Backward");
             if (armed==1){
              val1+=1;
              val3-=1;
             }
              break;
           case 100:Serial.println("roll left");
            if (armed==1){
              val2+=1;
              val4-=1;
            }
              break;
            case 102:Serial.println("roll left");
             if (armed==1){
              val2-=1;
              val4+=1;
             }
              break;
          
              case 110: //m
              Serial.println("Lift-5");
               if (armed==1){
                  thrust += 5;
               }
              break;
             case 109: //n
             Serial.println("Lift+5");
              if (armed==1){
                 thrust -= 5;
              }
              
              break;
            
            default: 
            // if nothing else matches, do the default
            break;
          }
         
    }  //end of switch statment
         
    }  //end if not newline
    
     // received a newline (linefeed) character
    else{
         
       
    } //else end
  
    /*print the integers for each motor*/
    //  Serial.print(val1);
     // Serial.print(": ");
    //  Serial.print(val2);
    //  Serial.print(": ");
    //  Serial.print(val3);
    //  Serial.print(": ");
     // Serial.print(val4);
    //  Serial.print('\n');
    //  Serial.print('\n');
      
      
      /* Only want to write an integer between 0 and 180 to the motor. */
      if (armed==1){
        if (val1 > -1 && val1 < 181 && val2 > -1 && val2 < 181 && val3 > -1 && val3 < 181 && val4 > -1 && val4 < 181)
          {
    
            // Write to Servo
            Serial.println("flying");
            myMotor1.write(val1);
            myMotor2.write(val2);
            myMotor3.write(val3);
            myMotor4.write(val4);
          } //if end
          
          // The value is not between 0 and 180.
          // We do not want write this value to
          // the motor.
          else
          {
            Serial.println("Value is NOT between 0 and 180");
            Serial.print('\n');
    
            // IT'S a TRAP!
            Serial.println("Error with the input");
          } //else end
      } //if armed end
           
     
}//main end
 



void i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}
uint8_t* i2cRead(uint8_t address, uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom(address, nbytes);
  for (uint8_t i = 0; i < nbytes; i++)
    data[i] = Wire.read();
  Wire.endTransmission();
  return data;
}
int readGyroX() { // This really measures the y-axis of the gyro
  uint8_t* data = i2cRead(gyroAddress, 0x1F, 2);
  return ((data[0] << 8) | data[1]);
}
int readGyroY() { // This really measures the x-axis of the gyro
  uint8_t* data = i2cRead(gyroAddress, 0x1D, 2);
  return ((data[0] << 8) | data[1]);
}
double getXangle() {
  double accXval = (double)readAccX();
  double accZval = (double)readAccZ();
  double angle = (atan2(accXval, accZval) + PI) * RAD_TO_DEG;
  return angle;
}
double getYangle() {
  double accYval = (double)readAccY() - zeroValue[1];
  double accZval = (double)readAccZ() - zeroValue[2];
  double angle = (atan2(accYval, accZval) + PI) * RAD_TO_DEG;
  return angle;
}
int readAccX() {
  uint8_t* data = i2cRead(adxlAddress, 0x32, 2);
  return (data[0] | (data[1] << 8));
}
int readAccY() {
  uint8_t* data = i2cRead(adxlAddress, 0x34, 2);
  return (data[0] | (data[1] << 8));
}
int readAccZ() {
  uint8_t* data = i2cRead(adxlAddress, 0x36, 2);
  return (data[0] | (data[1] << 8));
}

void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
       error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm> outMax) ITerm= outMax;
      else if(ITerm< outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
    
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}
