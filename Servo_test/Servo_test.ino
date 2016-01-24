//---- Written by: Yuting Chan for 4A project
//---- The following codes allow you to control 2 servos providing 
//---- them with the same angle input from the serial monitor.
//---- The pins used is PIN 9 and PIN 8

#include <Servo.h>

String inString = "";    // string to hold input
Servo myservo;
Servo myservo2;
//int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(8);
  Serial.begin(9600);
//  Serial.write("Power On");
}

void loop(){
  while (Serial.available() > 0)
  {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string,
    // then the string's value:
    if (inChar == '\n') {
      Serial.print(inString);
      myservo.write(inString.toInt());
      myservo2.write(inString.toInt());
      Serial.flush();
      // clear the string for new input:
      inString = "";
    }
  }
}


//void loop() {
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//}
