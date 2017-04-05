/*
 * How to configure and pair two HC-05 Bluetooth Modules
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 * 
 *                 == SLAVE CODE ==
 */

#include <SoftwareSerial.h>

#define errorLED 10
#define dataLED 9
#define statusLED 8
#define button 7

int state = 20;
int buttonState = 0;

void setup() {
  pinMode(button, INPUT);
  pinMode(dataLED, OUTPUT);
  pinMode(errorLED, OUTPUT);
  pinMode(statusLED, OUTPUT);
  //digitalWrite(statusLED, LOW);
  //myServo.attach(9);
  Serial.begin(38400); // Default communication rate of the Bluetooth module
}

void loop() {
 digitalWrite(statusLED,LOW);
 if(Serial.available() > 0){ // Checks whether data is comming from the serial port    
    state = Serial.read(); // Reads the data from the serial port
   digitalWrite(statusLED, HIGH);
 }
Serial.print(state);
Serial.print('\n');
 // Controlling the servo motor
  if (state == '1') {
  digitalWrite(dataLED, HIGH); // LED ON
  state = 0;
 }
 else if (state == '0') {
  digitalWrite(dataLED, LOW); // LED ON
  state = 0;
 }
 else 
 digitalWrite(errorLED, HIGH);
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
