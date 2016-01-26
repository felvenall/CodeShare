// Code taken from Mr. John K and modified/amended by Viktor Kapetanovic for 4A06 Capstone 

#include <SPI.h>
#include <avr/pgmspace.h>
#include <math.h>

// Define all registers
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

extern const unsigned short firmware_length;
extern const char firmware_data[];

// pointers to X and Y readings
volatile byte xydat[4];
int16_t *x = (int16_t *) &xydat[0];
int16_t *y = (int16_t *) &xydat[2];

// ----------------------------------------------------
int ncs = 10;
void setup() { 
  Serial.begin(9600);  // set bit rate
  
  pinMode(ncs, OUTPUT);  // serial port enable
  
  SPI.begin();  // begin communication
  
  // set clock polarity (CPOL = 1) and phase (CPHA = 1)
  // active state is 1 and idle is 0
  // data read on rising edge, data output on falling edge
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);  // sets the order of the bits shifted out of and into the SPI bus
  SPI.setClockDivider(8); // SPI clock 1/8 of system frequency 
  
  performStartup();  
}

// ---------------------------------------------------  
// main loop
float xprev = 0;
float yprev = 0;
void loop() {
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
      
   Serial.println("Angle, " + String(phi) + ", Speed, " + String(speeed) + ", X, " + String(*x) + ", Y, " + ", " + String(*y));
    
    xprev = *x;
    yprev = *y;
    delay(300);  // delay between readings
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
// initialization
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
// upload firmware
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

