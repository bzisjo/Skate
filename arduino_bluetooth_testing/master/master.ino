/*
 * How to configure and pair two HC-05 Bluetooth Modules
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 * 
 *                 == MASTER CODE ==
 */

#define ledPin 9 //
#define statusLED 6
int state = 0;
int pot = 0;
int trans_status = 0;
void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(statusLED, OUTPUT);
  //digitalWrite(ledPin, LOW);
  Serial.begin(38400); // Default communication rate of the Bluetooth module
}

void loop() {
 if(Serial.available() > 0){ // Checks whether data is comming from the serial port    
    trans_status = 1;
    state = Serial.read(); // Reads the data from the serial port
 }
 if(trans_status)
 digitalWrite(statusLED, HIGH);
 // Controlling the LED
//Serial.print(state,DEC);
//Serial.print('\n');
 if (state == '1') {
// if (trans_status == 1) {  
  digitalWrite(ledPin, HIGH); // LED ON
  state = 0;
 }
 else if (state == '0') {
//else if (trans_status == 0) {
  digitalWrite(ledPin, LOW); // LED OFF
  state = 0;
 }
 // Reading the potentiometer
 pot = digitalRead(2);
 //int potValueMapped = map(potValue, 0, 1023, 0, 255);

 if(pot){
 Serial.write('1'); // Sends potValue to servo motor
// Serial.print('\n');
}
 else{
 Serial.write('0');
// Serial.print('\n');
}
 delay(10);
 
}
