/*
 * How to configure and pair two HC-05 Bluetooth Modules
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 * 
 *                 == SLAVE CODE ==
 */

#include <SoftwareSerial.h>
#define ledPin 9
#define button 8
#define error 10
//Servo myServo;
int state = 20;
int buttonState = 0;

void setup() {
  pinMode(button, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(error, OUTPUT);
  //myServo.attach(9);
  Serial.begin(38400); // Default communication rate of the Bluetooth module
}

void loop() {
 if(Serial.available() > 0){ // Checks whether data is comming from the serial port    
    state = Serial.read(); // Reads the data from the serial port
 }
 // Controlling the servo motor
  if (state == '1') {
  digitalWrite(ledPin, LOW); // LED ON
  state = 0;
 }
 else if (state == '0') {
  digitalWrite(ledPin, LOW); // LED ON
  state = 0;
 }
 else 
 digitalWrite(error, HIGH);
 //Serial.print(state);
 //Serial.print('\n');

 delay(10);
 
 // Reading the button
 buttonState = digitalRead(button);
 if (buttonState == HIGH) {
   Serial.write('1'); // Sends '1' to the master to turn on LED
 }
 else {
  Serial.write('0');
 }  
}
