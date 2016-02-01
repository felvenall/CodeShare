/////----- Written by: 4A06 SprungKraft Group Eng Phys McMaster
/////----- This is the main structure for the code of SprungKraft

//---IMPT------ Standardised Reference Point-----IMPT---//
//Sensors   - 0   1   2   3   4   5   6   7   8   9 CLOCKWISE NUMBERING
//Angle     - 0   36  72  108 144 180 216 252 288 324

#include <Servo.h>


/////////////intiation variables for SERVO control////////////
#define servoUpDownPin1 5
#define servoUpDownPin2 3
#define servoRotationPin 7
Servo myservoAngle;
Servo myservoUpDown1;
Servo myservoUpDown2;

////////////intiation variables for ESC control///////////

#define pwm 11
Servo driveMotor; // create motor object
volatile int setVal; // analog read value

////////////intiation variables for EDGE DETECTION TO WHEEL ANGLE///////////////////
const int numReadings = 10; //---> change to number of sensors you want to read out of 10
int readings[numReadings];  //---> Array to keep track of sensors first analogue readings
int readings2[numReadings]; //---> Array to keep track of sensors analogue readings
int readIndex = 0; //----> Variable used for indexing
int diffReadings[numReadings]; //----> Array used to keep track of differences between current an previous readings
int edgeTrack[numReadings]={0}; //----> Array to keep track of which sensors is indicated to be close to edge
int angleTrack[10]={0,36,72,108,144,180,216,252,288,324}; //---> Array to retrieving sensor's angle data
int angleTable; //-----> Variable for angle of table relative to reference sensor 0
int anglePuck; //-----> Variable for angle of movement of puck relative to reference sensor 0
int angleBounce; //-----> Variable for angle that the puck should move in
int wheelAngle; //-----> Variable for angle that wheel should change to
int wheelDir; //---> Variable for whether wheels hould move forward or reverse: 1 for forward, 0 for backward
//int setVal=7;
//////////////////////////////////////
//A0-A5, A6-A11 (4,6,8,9,10,12)
// pointers to X and Y readings
volatile byte xydat[4];
int16_t *x = (int16_t *) &xydat[0];
int16_t *y = (int16_t *) &xydat[2];

int ncs = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ///////--- To retrieve first set of readings for edge ---/////////
  //////--- CAN DO TESTING HERE----/////// ********* To add code for sensor testing
  while ( readIndex < numReadings){
  readings[readIndex] = analogRead(readIndex);
  readIndex = readIndex+1;
  }
  readIndex=0;

  //////servos pins setup////////
  myservoAngle.attach(servoRotationPin);
  myservoUpDown1.attach(servoUpDownPin1);
  myservoUpDown2.attach(servoUpDownPin2);
  //Serial.write("Setup Done");
  driveMotor.attach(pwm);
  angleBounce=0;
  //wheelAngle=0;
  wheel();
  servoUp();
  servoRotation();
  driveMotor.write(0);
  driveMotor.write(90);
  delay(2000);
  drive();
}

//The 2nd loop in the main loop is the code to read and analyse the difference between current and previous reading 
// This codes compare the current and previous edge sensor readings and whenever there is a difference of
// more than a 100, it will set a value in the edgeTrack array, to say which sensor is high. Such as
// when sensor 4 has a 100 difference, it will set the edgeTrack index 4 to 1.
// Following this, whenever there is a sensor that is set to 1 in edgeTrack, it will also check if
// the current reading for the sensor is above 600 (which indicate that the sensor is very close to the edge).
// If it is, it will run the function: angle_change().

void loop() {
  //Get angle of table, reference to sensor 1
  //angleTable=readAngle();
  //servoUp();
  for (int i=0;i<numReadings;i=i+1){
    readings2[i] = analogRead(i);
  }
  for (int i=0;i<numReadings;i=i+1){
    diffReadings[i]=readings2[i]-readings[i];
    //Serial.println(diffReadings[i]);
    if (diffReadings[i]>200){ //double check, if the puck is really really moving slowly to the edge, maybe no changes will be >100
      edgeTrack[i]=1;
    }
    if (edgeTrack[i]==1){
      if (readings2[i]>400){
        driveMotor.write(90);
        delay(1000);
        servoUp();
        angleTable=angle_change();
        for (int k=0;k<numReadings;k=k+1){
          edgeTrack[k]=0;} 
          //Serial.println(angleTable);
          //---------continue as edge is close-------------//
          
          //Get angle of puck, reference to sensor 1 (VIKTOR'S CODE)
          //anglePuck=ADNS_main();

          anglePuck=angleBounce;
          //Serial.print(angleTable);
          //Serial.print(',');
          //get bouncing angle

          angleBounce=bounceAngle();
          //function to correct angle and wheel direction for wheel angle limitation, amend according to angle limitation
          wheel();
          servoRotation();// rotate top servo for wheel angle
          //function to run motor with direction given by wheelDir (JON'S CODES)
          //setVal= motorCal();
          //driveMotor.write(setVal);  // send speed command to ESC  
          //Drop down wheel
          servoDown();
          drive();
          delay(1000);
    }
    }
  }
}

void drive(){
  if (wheelDir==1){//forward
   //0-90 or 90 to 180, according to motor calibration 
   driveMotor.write(95);
  }
  else{//backwards
    //0-90 or 90-180, according to motor calibration
    driveMotor.write(81);
  }
}

///----- This is the function to return angle of table with reference to sensor 0.
///----- Based on testing, when there two sensors that have differences of 100 before any sensor hits 600 in reading,
///----- the angle will be approximately halfway between that two sensors.
///----- If only one sensor has difference of 100 before any hits 600 in reading, the angle will be approximately 
///----- be that  of the sensor.
///----- If three sensors have difference of 100 before any hits 600 in reading, the angle will be approximately 
///----- that of the sensor in the middle.
int angle_change(){
   for (int j=0;j<numReadings;j=j+1){ // goes through edgeTrack and see if any is high
    if (edgeTrack[j]==1){
/// if any is high, check the next three sensors readings, except for the second last and last index
/// if the first index is high, need to check for sensors 8 and 9
      if (j==0){ // Sensor 0 is high
        if (edgeTrack[numReadings-1]==1){
          if (edgeTrack[1]==1){ // sensor 9,0,1
            return angleTrack[j];             
          }
          if (edgeTrack[numReadings-2]==1){// sensor 8,9,0
            return angleTrack[numReadings-1];                
          }
          else{// between sensor 0 and 9
            return 360-18;
          }
        }
        else{
          if (edgeTrack[1]==1 && edgeTrack[2]==1){//Sensor 0,1,2
            return angleTrack[1];                          
          }
           if (edgeTrack[1]==1 && edgeTrack[2]==0){//Sensor 0,1
            return angleTrack[0]+18;                          
          }         
        }
      }
        
      if (j==numReadings-1){//this is so that it does not read beyond index //sensor 9 
        return angleTrack[j];
        }

      if (j==numReadings-2){ 
        if (edgeTrack[j+1]==1){ //sensor 8,9
          return angleTrack[j]+18;
        }
        else{//sensor 8
          return angleTrack[j];
          }
          }

      else{
        if (edgeTrack[j+1]==1 && edgeTrack[j+2]==0){
          return angleTrack[j]+18;
          }
          if(edgeTrack[j+1]==1 && edgeTrack[j+2]==1){        
            return angleTrack[j+1];
            }
            else{
              return angleTrack[j];
              }
              }
    }
   }
}

//code to calculate the bounce angle indicating movement direction and table edge
int bounceAngle(){
  int bounce;
  if (anglePuck<angleTable){
    bounce= angleTable-180+(angleTable-anglePuck);
  }
  else{
    bounce= angleTable+180-(anglePuck-angleTable);
  }
  if (bounce<0){
    bounce=360-bounce;
  }
  if (bounce>=360){
    bounce=bounce-360;
  }
  return bounce;
}

void wheel(){
  //wheel can go limited angle from between (angle  to 288)
  if (angleBounce>200){
   angleBounce= angleBounce-180;
   wheelDir=1;//forward
  }
  //if (angleBounce<0){
  //  angleBounce= angleBounce+180;
  //  wheelDir=0;
  //}
  if (angleBounce>=0 && angleBounce<=200){
    wheelDir=0; //backwards
  }
}

void servoUp(){
  myservoUpDown1.writeMicroseconds(1600);
  myservoUpDown2.writeMicroseconds(1600);
}

void servoDown(){
  myservoUpDown1.writeMicroseconds(1200);
  myservoUpDown2.writeMicroseconds(1200);
}

void servoRotation(){
  int startAngle=0; //Correspond to 2400us update according to calibration Max
  int endAngle=200; //Correspond to 600us
  int caliAngle=(((angleBounce-endAngle)/(startAngle-endAngle))*(2400-600))+600;
  myservoAngle.writeMicroseconds(caliAngle);
}

//------- function to determine motor speed input-----//
int motorCal(){
  //GET SPEED FROM mouse sensor//
  //Calibrate speed
  if (wheelDir==1){
   //0-90 or 90 to 180, according to motor calibration 
   
  }
  else{
    //0-90 or 90-180, according to motor calibration
    
  }
}



