
/////----- Written by: 4A06 SprungKraft Group Eng Phys McMaster
/////----- This is the main structure for the code of SprungKraft

//---IMPT------ Standardised Reference Point-----IMPT---//
//Sensors   - 0   1   2   3   4   5   6   7   8   9 CLOCKWISE NUMBERING
//Angle     - 0   36  72  108 144 180 216 252 288 324

#include<Servo.h>

/////////////intiation variables for SERVO control////////////
#define servoUpDownPin1 9
#define servoUpDownPin2 10
#define servoRotationPin 8

Servo myservoAngle;
Servo myservoUpDown1;
Servo myservoUpDown2;

////////////intiation variables for EDGE DETECTION TO WHEEL ANGLE///////////////////
const int numReadings = 5; //---> change to number of sensors you want to read out of 10
int readings[numReadings];  //---> Array to keep track of sensors first analogue readings
int readings2[numReadings]; //---> Array to keep track of sensors analogue readings
int diffReadings[numReadings]; //----> Array used to keep track of differences between current an previous readings
int edgeTrack[numReadings]={0}; //----> Array to keep track of which sensors is indicated to be close to edge
int angleTrack[10]={0,36,72,108,144,180,216,252,288,324}; //---> Array to retrieving sensor's angle data
int angleTable; //-----> Variable for angle of table relative to reference sensor 0
int anglePuck; //-----> Variable for angle of movement of puck relative to reference sensor 0
int angleBounce; //-----> Variable for angle that the puck should move in
int wheelAngle; //-----> Variable for angle that wheel should change to
int wheelDir; //---> Variable for whether wheels hould move forward or reverse: 1 for forward, 0 for backward
//////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  ///////--- To retrieve first set of readings for edge ---/////////
  //////--- CAN DO TESTING HERE----/////// ********* To add code for sensor testing
  for (int i=0;i<numReadings;i=i+1){
  readings[i] = analogRead(i);
  }

  //////servos pins setup////////
  myservoAngle.attach(servoRotationPin);
  myservoUpDown1.attach(servoUpDownPin1);
  myservoUpDown2.attach(servoUpDownPin2);
  //Serial.write("Setup Done");
}

void loop() {
  // put your main code here, to run repeatedly:

  //Get angle of table, reference to sensor 1
  //angleTable=readAngle();
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
        angleTable=angle_change();
        for (int k=0;k<numReadings;k=k+1){
          edgeTrack[k]=0;} 
          //Serial.println(angleTable);
          //---------continue as edge is close-------------//
          //Get angle of puck, reference to sensor 1 (VIKTOR'S CODE)
          anglePuck=25;
          Serial.print(angleTable);
          Serial.print(',');
          //get bouncing angle
          angleBounce=bounceAngle();
          Serial.println(angleBounce);
          delay(1000);
          
          //function to correct angle and wheel direction for wheel angle limitation, amend according to angle limitation
          //wheel();
          //myservoAngle.writeMicroseconds(angleBounce);
          
          //function to run motor with dirction given by wheelDir (JON'S CODES)
          
          //Drop down wheel
          //servoDown();
          
    }
    }
  }
}

/////////////**** Will need to amend and improve: code to read the sensors repeatedly and return angle////////////
////////////---- This is the code to read and analyse the difference between current and previous reading 
///////////----- (TIMING is very important for this)
//////////------ This codes compare the current and previous edge sensor readings and whenever there is a difference of
/////////------- more than a 100, it will set a value in the edgeTrack array, to say which sensor is high. Such as
/////////------- when sensor 4 has a 100 difference, it will set the edgeTrack index 4 to 1.
////////-------- Following this, whenever there is a sensor that is set to 1 in edgeTrack, it will also check if
////////-------- the current reading for the sensor is above 600 (which indicate that the sensor is very close to the edge).
///////--------- If it is, it will run the function: angle_change().
//int readAngle(){
//  //delay(100);
//  for (int i=0;i<numReadings;i=i+1){
//    readings2[i] = analogRead(i);
//  }
//  for (int i=0;i<numReadings;i=i+1){
//    diffReadings[i]=readings2[i]-readings[i];
//    //Serial.println(diffReadings[i]);
//    if (diffReadings[i]>200){ //double check, if the puck is really really moving slowly to the edge, maybe no changes will be >100
//      edgeTrack[i]=1;
//    }
//    if (edgeTrack[i]==1){
//      if (readings2[i]>600){
//        angleTable=angle_change();
//        for (int k=0;k<numReadings;k=k+1){
//          edgeTrack[k]=0;} 
//          //Serial.println(angleTable);
//    }
//    }
//  }
//}

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
    bounce=360+bounce;
  }
  if (bounce>=360){
    bounce=bounce-360;
  }
  return bounce;
}


void serialprint(){
    for (int i=0;i<numReadings;i=i+1){
    Serial.print(readings2[i]);//-readings[i]);
    readings[i]=readings2[i];
    if (i<(numReadings-1)){
      Serial.print(',');
    }
    }
    Serial.println('.');
    delay(100);
}


void wheel(){
  //wheel can go limited angle from between sensor 2 to 8 (angle 72 to 288)
  if (angleBounce>280){
   angleBounce= angleBounce-180;
   wheelDir=0;
  }
  if (angleBounce<72){
    angleBounce= angleBounce+180;
    wheelDir=0;
  }
  if (angleBounce>=72 && angleBounce<=280){
    wheelDir=1;
  }
}

void servoUP(){
  myservoUpDown1.write(90);
  myservoUpDown2.write(90);
}

void servoDown(){
  myservoUpDown1.write(0);
  myservoUpDown2.write(0);
}
