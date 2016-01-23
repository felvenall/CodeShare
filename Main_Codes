#include<Servo.h>

/////////////intiation variables for SERVO control////////////
#define servoUpDownPin1 9
#define servoUpDownPin2 10
#define servoRotationPin 8

Servo myservoAngle;
Servo myservoUpDown1;
Servo myservoUpDown2;

////////////intiation variables for EDGE DETECTION TO WHEEL ANGLE///////////////////
const int numReadings = 3;
int readings[numReadings];
int readIndex = 0;
int readings2[numReadings];
int inputPin=0;
int diffReadings[numReadings];
int edgeTrack[numReadings]={0};
//Sensors   - 0   1   2   3   4   5   6   7   8   9 CLOCKWISE NUMBERING
//Angle     - 0   36  72  108 144 180 216 252 288 324
int angleTrack[10]={0,36,72,108,144,180,216,252,288,324};
int angleTable;
int anglePuck;
int angleBounce;
int wheelAngle;
int wheelDir; //1 for forward, 0 for backward
//////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  ///////first readings for edge/////////
  while ( readIndex < numReadings){
  readings[readIndex] = analogRead(readIndex);
  readIndex = readIndex+1;
  }
  readIndex=0;

  //////servos pins setup////////
  myservoAngle.servoRotationPin;
  myservoUpDown1.servoUpDownPin1;
  myservoUpDown2.servoUpDownPin2;
  //Serial.write("Setup Done");
}

void loop() {
  // put your main code here, to run repeatedly:

  //Get angle of table, reference to sensor 1
  angleTable=readAngle();
  
  //Get angle of puck, reference to sensor 1 (VIKTOR'S CODE)
  //anglePuck=;
  
  //get bouncing angle
  angleBounce=bounceAngle();

  //function to correct angle and wheel direction for wheel angle limitation, amend according to angle limitation
  wheel(); 
  myservoAngle.write(angleBounce);
  
  //function to run motor with dirction given by wheelDir (JON'S CODES)
  
  //Drop down wheel
  servoDown();
}

/////////////IMPT TO IMPROVE: code to read the sensors repeatedly and return angle//////////// 
int readAngle(){
  delay(100);
    while ( readIndex < numReadings){
    readings2[readIndex] = analogRead(readIndex);
    readIndex = readIndex+1;
  }
  readIndex=0;
  for (int i=0;i<numReadings;i=i+1){
    diffReadings[i]=readings2[i]-readings[i];
    if (diffReadings[i]>100){ //double check, if the puck is really really moving slowly to the edge, maybe no changes will be >100
      edgeTrack[i]=1;
    }
    if (edgeTrack[i]==1){
      if (readings2[i]>600){
        return angle_change();
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

//function to return angle of table
int angle_change(){
   for (int j=0;j<numReadings;j=j+1){
    if (edgeTrack[j]==1){
      
      if (j==numReadings-1){
        if (edgeTrack[0]==1 && edgeTrack[1]==0){
          return angleTrack[j]+18;          
        }
        if(edgeTrack[0]==1 && edgeTrack[1]==1){
            return angleTrack[0];
            }
            else{
              return angleTrack[j];
              }
              }
              
      if (j==numReadings-2){
        if (edgeTrack[j+1]==1 && edgeTrack[0]==0){
          return angleTrack[j]+18;          
        }
        if(edgeTrack[j+1]==1 && edgeTrack[0]==1){
            return angleTrack[j+1];
            }
            else{
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
  myservoUpDown1.write(100);
  myservoUpDown2.write(100);
}

void servoDown(){
  myservoUpDown1.write(0);
  myservoUpDown2.write(0);
}
