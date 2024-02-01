
//This code controls all the avionics sub-system operations

//Note in order for code to compile line 39 and 217 must be commented out as they use Teensy 4.1 specific hardware commands 

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MPL3115A2.h>
#include <SPI.h>
#include <SD.h>
#include <stdint.h>
#include <CytronMotorDriver.h>
#include <Servo.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55); // create IMU sensor object called bno
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();// create altimeter sensor object called baro

Servo actuator; // create a servo object named "actuator"
CytronMD motor(PWM_DIR, 28,27); // configure Cytron motor driver pins

File RocketTelemetry; // create file named RocektTelemetry to store sensor data

//global Variables 
unsigned long LaunchTime; //Millis value at the time of launch
int EjectSwitch = 32 ; //payload ejection switch
float KFalt = 0; // define Kalman filtered altitude variable
float KFalt_prev = 0; // define previous Kalman filtered altitude variable
float KFacc = 0;  // define Kalman filtered acceleration variable
float KFacc_prev = 0; // define previous Kalman filtered acceleration variable
bool apogee_reached = 0; // define variable that determines whether apogee has been reached 
float alt = 0; //altitude data
float tempC; // temperature data
float acc = 0;  //acceleration data
float ori_x; //x-axis orientation data
float ori_y; //y-axis orientation data
float ori_z; //z-axis orientation data
const int chipSelect = BUILTIN_SDCARD; // assign chipSelect pin for built in SD card reader

   
//function to change oxidsier flow control motor speed over time
void OxidiserFlowControl (){
      
      unsigned long CurrentMillis = millis();
      
      if((CurrentMillis - LaunchTime) >= 1000 && (CurrentMillis - LaunchTime) < 2000){
        motor.setSpeed(125);}
  
      else if ((CurrentMillis - LaunchTime) >= 2000 && (CurrentMillis - LaunchTime) < 3000){
        motor.setSpeed(118);}
  
      else if ((CurrentMillis - LaunchTime) >= 3000 && (CurrentMillis - LaunchTime) < 4000){
        motor.setSpeed(111);}
  
      else if ((CurrentMillis - LaunchTime) >= 4000 && (CurrentMillis - LaunchTime) < 5000){
        motor.setSpeed(102);}
  
      else if ((CurrentMillis - LaunchTime) >= 5000 && (CurrentMillis - LaunchTime) < 6000){
        motor.setSpeed(95);}
  
      else if ((CurrentMillis - LaunchTime) >= 6000 && (CurrentMillis - LaunchTime) < 7000){
        motor.setSpeed(88);}
  
      else if ((CurrentMillis - LaunchTime) >= 7000 && (CurrentMillis - LaunchTime) < 8000){
        motor.setSpeed(81);}
  
      else if ((CurrentMillis - LaunchTime) >= 8000 && (CurrentMillis - LaunchTime) < 9000){
        motor.setSpeed(72);}
        
      else if ((CurrentMillis - LaunchTime) >= 9000 && (CurrentMillis - LaunchTime) < 10000){
        motor.setSpeed(72);}
  
      else if ((CurrentMillis - LaunchTime) >= 10000 && (CurrentMillis - LaunchTime) < 11500){
        motor.setSpeed(65);}
  
      else if ((CurrentMillis - LaunchTime) >= 11500){
        motor.setSpeed(0);}
     }


//function to eject payload if ejection switch has been activated   
void PayloadEject (){
     unsigned long  CurrentMillis2 = millis();
     unsigned long  PreviousMillis2 = LaunchTime; 
     int val = digitalRead(EjectSwitch);
     if(val == HIGH){
        digitalWrite(11, HIGH); // activate P16-S actuator to eject payload 
        actuator.writeMicroseconds(2000); // retract L12-R actuator
        if (CurrentMillis2 - PreviousMillis2  >= 5000){  // wait 5 seconds to ensure payload ejection mechanism is complete
          digitalWrite(11,LOW);
        }
      PreviousMillis2 = CurrentMillis2;
     }
}
   
//function to read data from sensors
void SensorRead () {
   
    //retrieve imu data 
     sensors_event_t event; 
     bno.getEvent(&event);
  
    //retrieve altitude and temperature data from altimeter  
     float alt = baro.getAltitude();
     float tempC = baro.getTemperature();
  
     //retrieve vertical accleration and orientation data from altimeter  
     float acc = event.acceleration.z; 
     float ori_x = event.orientation.x;
     float ori_y = event.orientation.y;
     float ori_z = event.orientation.z; 
  
     float KFalt_prev = KFalt; // replace previous kalman filtered altitude with current value
     float KFacc_prev = KFacc; // replace previous kalman filtered acceleration with current value
}

//function to apply Kalman filter to data to remove noise
void KalmanFilter (){

      static const float R_alt = 1e-1; // define altitude measurement noise covariance 
      static const float R_acc = 1e-1; // define acceleration measurement noise covariance 
      static const float Q = 1e-1; // degine process noise covariance 
      static float P_alt = 0; // define altitude estimate error covariance
      static float P_acc = 0; // define acceleration estimate error covariance
        
      static float K_alt; // define Kalman gain for altitude filter
      static float K_acc; // define Kalman gain for accleration filter

      //altitude Kalman filter 
      KFalt = KFalt_prev; //calculate a priori state estimate
      P_alt = P_alt + Q; // calculate a priori error covariance
      K_alt = P_alt/(P_alt+R_alt); //calculate Kalman gain
      KFalt = KFalt + K_alt*(alt - KFalt); //calculate a posteriori estimate
      P_alt = (1-K_alt)*P_alt; //calculate a posteriori error covariance


      //acceleration Kalman filter 
      KFacc = KFacc_prev; //calculate a priori state estimate
      P_acc = P_acc + Q; // calculate a priori error covariance
      K_acc = P_acc/(P_acc+R_acc); //calculate Kalman gain
      KFacc = KFacc + K_acc*(acc - KFacc); //calculate a posteriori estimate
      P_acc = (1-K_acc)*P_acc; //calculate a posteriori error covariance


      
}


//function to detect apogee, trigger drogue parachute deployment and open payload bay doors 
void ApogeeEvent (){
        unsigned long  CurrentMillis3 = millis();
        unsigned long  PreviousMillis3 = LaunchTime; 
        if (KFalt < KFalt_prev && apogee_reached == 0)
        {  
          apogee_reached = 1;
          digitalWrite (9, HIGH); // trigger drouge e-match 
          if (CurrentMillis3 - PreviousMillis3  >= 2000){   // wait for 2 seconds to ensure drogue is deployed 
          digitalWrite (9, LOW);
          actuator.writeMicroseconds(1000); // activate L12-R actuator to open payload doors 
          }
          PreviousMillis3 = CurrentMillis3; 
        }
}


void MainRecovery (){
        unsigned long  CurrentMillis4 = millis();
        unsigned long  PreviousMillis4 = LaunchTime; 
        if (apogee_reached == 1 && KFalt <= 457){
          digitalWrite(10, HIGH); // trigger main recovery e-match
          if (CurrentMillis4 - PreviousMillis4  >= 2000){  // wait for 2 seconds to ensure drogue is deployed 
          digitalWrite (10, LOW);
          }
          PreviousMillis4 = CurrentMillis4; 
        }
}
   
//function to store sensor data on SD card
void DataLogger (){
      
  RocketTelemetry = SD.open("RocketTelemetry.txt", FILE_WRITE); //create text file to log rocket telemetry in
      
      // Write sensor data to SD card 
      if (RocketTelemetry) {
           RocketTelemetry.print(KFalt); 
           RocketTelemetry.print(",");        
           RocketTelemetry.print(KFacc); 
           RocketTelemetry.print(",");        
           RocketTelemetry.print(tempC);                             
           RocketTelemetry.print(",");                              
           RocketTelemetry.println(ori_x);           
           RocketTelemetry.print(",");      
           RocketTelemetry.print(ori_y); 
           RocketTelemetry.print(",");       
           RocketTelemetry.print(ori_z); 
           RocketTelemetry.println();   
           RocketTelemetry.close();    
          }
}
        
void setup() {

  pinMode(EjectSwitch, INPUT); // configure payload ejection switch
  
  //configure relay control pins 
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  // initialise IMU
  bno.begin();
  bno.setExtCrystalUse(true);

  baro.begin(); //initialise altimeter
    
  SD.begin(BUILTIN_SDCARD); // intialise SD card 
  
  actuator.attach(29); // confiure L12-R actuator to pin 29 (PWM)
  actuator.writeMicroseconds(2000); // give the actuator a 2ms pulse to ensure the arm is retracted before launch
  delay(5000); // 5s delay to allow the arm to retract

  // detect whether rocket has been launched  
  while (alt == 0 && apogee_reached == 0){
    alt = baro.getAltitude();
    LaunchTime = millis(); }

  motor.setSpeed(132); // turn on oxidiser control motor to max. speed 
}

void loop() {

   OxidiserFlowControl();
   PayloadEject();
   SensorRead();
   KalmanFilter();
   ApogeeEvent ();
   MainRecovery();
   DataLogger ();
}
