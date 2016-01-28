
/////----- Written by: 4A06 SprungKraft Group Eng Phys McMaster
/////----- This is the main structure for the code of SprungKraft

//---IMPT------ Standardised Reference Point-----IMPT---//
//Sensors   - 0   1   2   3   4   5   6   7   8   9 CLOCKWISE NUMBERING
//Angle     - 0   36  72  108 144 180 216 252 288 324

#include <Servo.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <math.h>

Servo myservoAngle;
Servo myservoUpDown1;
Servo myservoUpDown2;

/////////////intiation variables for SERVO control////////////
#define servoUpDownPin1 9
#define servoUpDownPin2 10
#define servoRotationPin 8

// Define all ADNS registers
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_SROM_Enable                          0x13
#define REG_LASER_CTRL0                          0x20
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62

// call ADNS firmware data
extern const unsigned short firmware_length;
extern const char firmware_data[];

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
//////////////////////////////////////

// pointers to X and Y readings
volatile byte xydat[4];
int16_t *x = (int16_t *) &xydat[0];
int16_t *y = (int16_t *) &xydat[2];

int ncs = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(ncs, OUTPUT);  // serial port enable
  
  SPI.begin();  // begin communication
  
  // set clock polarity (CPOL = 1) and phase (CPHA = 1)
  // active state is 1 and idle is 0
  // data read on rising edge, data output on falling edge
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);  // sets the order of the bits shifted out of and into the SPI bus
  SPI.setClockDivider(8); // SPI clock 1/8 of system frequency 
  
  performStartup();  // power up ADNS
  
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
}

void loop() {
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
          anglePuck=ADNS_main();
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

//The 2nd loop in the main loop is the code to read and analyse the difference between current and previous reading 
//(TIMING is very important for this)
// This codes compare the current and previous edge sensor readings and whenever there is a difference of
// more than a 100, it will set a value in the edgeTrack array, to say which sensor is high. Such as
// when sensor 4 has a 100 difference, it will set the edgeTrack index 4 to 1.
// Following this, whenever there is a sensor that is set to 1 in edgeTrack, it will also check if
// the current reading for the sensor is above 600 (which indicate that the sensor is very close to the edge).
// If it is, it will run the function: angle_change().



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
  myservoUpDown1.writeMicroseconds(900);
  myservoUpDown2.writeMicroseconds(900);
}

void servoDown(){
  myservoUpDown1.writeMicroseconds(600);
  myservoUpDown2.writeMicroseconds(600);
}

// ---------------------------------------------------  
// main ADNS loop
float xprev = 0;
float yprev = 0;
void ADNS_main() {
    float t_start = micros();
    UpdatePointer();
    float t_final = micros();
    float phi = round(atan2(abs(float(*y)), abs(float(*x)))*180/3.14159265);
    float xspeeed = (abs(*x) - abs(xprev))/((t_final - t_start)); 
    float yspeeed = (abs(*y) - abs(yprev))/((t_final - t_start));
    float speeed = sqrt(sq(xspeeed) + sq(yspeeed));
    
    if ((float(*x) < 0) ^ ((-1)*float(*y) < 0)){
      if (float(*x) < 0){
        phi = 180 - phi;
      }
      else{
        phi = 360 - phi;
      }
    }
    else if ((float(*x) < 0) && ((-1)*float(*y) < 0)){
      phi = 180 + phi;
    }
    else{
      phi = phi;
    }
      
   //Serial.println("Angle, " + String(phi) + ", Speed, " + String(speeed) + ", X, " + String(*x) + ", Y, " + ", " + String(*y));
    
    xprev = *x;
    yprev = *y;
    delay(300);  // delay between readings
    
    return phi
 }

// ----------------------------------------------------
// get X and Y readings
void UpdatePointer(void){
    digitalWrite(ncs,LOW);
    xydat[0] = (byte)adns_read_reg(REG_Delta_X_L);
    xydat[1] = (byte)adns_read_reg(REG_Delta_X_H);
    xydat[2] = (byte)adns_read_reg(REG_Delta_Y_L);
    xydat[3] = (byte)adns_read_reg(REG_Delta_Y_H);
    digitalWrite(ncs,HIGH);     
    }

// ----------------------------------------------------
// set NCS low to activate serial port
void adns_com_begin(){
  digitalWrite(ncs, LOW);
}

// ----------------------------------------------------
// set NCS high to ignore inputs (tri-stated)
void adns_com_end(){
  digitalWrite(ncs, HIGH);
}

// ----------------------------------------------------
// read register address
byte adns_read_reg(byte reg_addr){
  adns_com_begin();
  
  SPI.transfer(reg_addr & 0x7f ); // send address of the register, with MSBit = 0 to indicate it's a read
  delayMicroseconds(100); // tSRAD
  
  byte data = SPI.transfer(0); // read data
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19); // tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

// ----------------------------------------------------
// write to register 
void adns_write_reg(byte reg_addr, byte data){
  adns_com_begin();
  
  SPI.transfer(reg_addr | 0x80 ); // send address of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(data); // send data
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

// ----------------------------------------------------
// ADNS initialization
void performStartup(void){
  adns_com_end(); // ensure that the serial port is reset
  adns_com_begin(); // ensure that the serial port is reset
  adns_com_end(); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  adns_write_reg(REG_Configuration_I, 0x44); // 3400 cpi
  delay(10);
  adns_write_reg(REG_Lift_Detection_Thr, 0x18);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  
  delay(1);
  Serial.println("Optical Chip Initialized");
  }

// ----------------------------------------------------
// upload ADNS firmware
void adns_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
  }
