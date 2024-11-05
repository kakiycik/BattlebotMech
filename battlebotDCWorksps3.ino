/*
  ESP32 PS3 Controller - LED Control Test
  esp32-ps3-led.ino
  Control three LEDs with PS3 Controller
  LED Hookup - LED1=4  LED2=16  LED3=15
  Requires ESP32-PS3 Library - https://github.com/jvpernis/esp32-ps3
  
  DroneBot Workshop 2023
  https://dronebotworkshop.com
*/

// Include PS3 Controller library
#include <Ps3Controller.h>
#include "driver/ledc.h"
//#include <ESP32Servo.h>
//#include <Servo.h> 

int leftX;
int leftY;
int rightx;
// Pin definitions
const int buzzerPin = 26; // Pin where the buzzer is connected
const int ledPin = 27;   // Pin where the LED is connected

const int motorPin1_m2 = 23;
const int motorPin2_m2 = 22;
const int motorPin1_m3 = 21;
const int motorPin2_m3 = 19;

const int motorPin1_m3 = 18; //weapon
const int motorPin2_m3 = 17;
// voltage divider
float voltage;
float R1 = 21000;
float R2 = 15000;
float tune_A = 1.0309; // tuning parameter 
const int vpPin = 36;  // Change this to your ADC pin



// Callback Function
void notify() {
   // Get Joystick value
  leftX = (Ps3.data.analog.stick.lx);
  leftY = (Ps3.data.analog.stick.ly);
  rightx = (Ps3.data.analog.stick.rx);

// Map the values from -128 to 128 into the range of 0 to 255
  //int mappedLeftX = map(leftX, -128, 128, 253, -253);
  int mappedLeftY = map(leftY, -128, 128, 256, -258);
  int mappedRightx = map(rightx, -128, 128, 253, -253);
  mappedLeftY = constrain(mappedLeftY, -255, 255);

  //Serial.print("X value = ");
  //Serial.println(mappedLeftX);
  Serial.print(" - leftY = ");
  Serial.println(mappedLeftY);
  //Serial.print(" - righty = ");
  //Serial.println(mappedRightY);
  ///
  if(mappedRightx>40){
      digitalWrite(motorPin1_m2, LOW);
      analogWrite(motorPin2_m2, abs(mappedLeftY));

      digitalWrite(motorPin2_m3, LOW);
      analogWrite(motorPin1_m3, abs(mappedLeftY));

    }else if(mappedRightx<-40){
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(motorPin1_m2, abs(mappedLeftY));

      digitalWrite(motorPin1_m3, LOW);
      analogWrite(motorPin2_m3, abs(mappedLeftY));
  }

  else if(mappedLeftY>40){
  digitalWrite(motorPin1_m2, LOW);
  analogWrite(motorPin2_m2, abs(mappedLeftY));

  digitalWrite(motorPin1_m3, LOW);
  analogWrite(motorPin2_m3, abs(mappedLeftY));

    if(mappedRightx>40){
      digitalWrite(motorPin1_m2, LOW);
      analogWrite(motorPin2_m2, abs(mappedLeftY));

      digitalWrite(motorPin2_m3, LOW);
      analogWrite(motorPin1_m3, abs(mappedLeftY));

    }else if(mappedRightx<-40){
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(motorPin1_m2, abs(mappedLeftY));

      digitalWrite(motorPin1_m3, LOW);
      analogWrite(motorPin2_m3, abs(mappedLeftY));
    }

  }else if (mappedLeftY<-40) {
  digitalWrite(motorPin2_m2, LOW);
  analogWrite(motorPin1_m2, abs(mappedLeftY));

  digitalWrite(motorPin2_m3, LOW);
  analogWrite(motorPin1_m3, abs(mappedLeftY));

    if(mappedRightx>40){
      digitalWrite(motorPin1_m2, LOW);
      analogWrite(motorPin2_m2, abs(mappedLeftY));

      digitalWrite(motorPin2_m3, LOW);
      analogWrite(motorPin1_m3, abs(mappedLeftY));

    }else if(mappedRightx<-40){
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(motorPin1_m2, abs(mappedLeftY));

      digitalWrite(motorPin1_m3, LOW);
      analogWrite(motorPin2_m3, abs(mappedLeftY));
    }

  }else {
  digitalWrite(motorPin1_m2, LOW);
  digitalWrite(motorPin2_m2, LOW);
  digitalWrite(motorPin1_m3, LOW);
  digitalWrite(motorPin2_m3, LOW);
  }
  // Map the readings between 0 and 180 and write them. With a deadband: check in monitor
//////////////////
    // Check if joystick was moved upward, indicating return to home (90 degrees)
//  if (leftY < -100) {

    // See if joystick was moved left or right, and in what direction. If moved, move servo in that direction
    /*if (leftX < -10 && servoPos < 180) {
      servoPos++;
      srvmtr.write(servoPos);
      delay(10);
    }
    if (leftX > 10 && servoPos > 0) {
      servoPos--;
      srvmtr.write(servoPos);
      delay(10);
    }
  }*/
/////////////////////////////
   // Print to Serial Monitor


  // Cross button - LED1 momentary control
  /*if (Ps3.event.button_down.cross) {
    Serial.println("Cross pressed");
    led1State = true;
    digitalWrite(LED1_PIN, led1State);
  }
  if (Ps3.event.button_up.cross) {
    Serial.println("Cross released");
    led1State = false;
    digitalWrite(LED1_PIN, led1State);
  }*/

  // Triangle Button - LED2 toggle control
 /* if (Ps3.event.button_down.square) {
    //Serial.println("square presssed");
    led3State = !led3State;
    digitalWrite(LED3_PIN, led3State);
  }*/
  if (Ps3.event.button_down.circle) {
  Serial.println("circle presssed");
  digitalWrite(motorPin1_m3, LOW);
  analogWrite(motorPin2_m3, 255);
  // state != state
    //digitalWrite(LED1_PIN, ledallState);
  }


  // Square Button - LED3 on
  /*if (Ps3.event.button_down.square) {
    Serial.println("Square pressed");
    led3State = true;
    digitalWrite(LED3_PIN, led3State);
  } 

  // Circle Button - LED3 off
  if (Ps3.event.button_down.circle) {
    Serial.println("Circle pressed");
    led3State = false;
    digitalWrite(LED3_PIN, led3State);
  }*/
// Analog sticks
 /* if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
       Serial.print("Moved the left stick:");
       Serial.print(" x="); Serial.print(Ps3.data.analog.stick.lx, DEC);
       Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ly, DEC);
       Serial.println();
       //myservo.write(Ps3.data.analog.stick.lx);
  }

  if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
       Serial.print("Moved the right stick:");
       Serial.print(" x="); Serial.print(Ps3.data.analog.stick.rx, DEC);
       Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ry, DEC);
       Serial.println();
      //myservo.write(Ps3.data.analog.stick.ry);
  }*/


}


// On Connection function
void onConnect() {
  // Print to Serial Monitor
  //Serial.println("Connected.");
}

void setup() {

  // Setup Serial Monitor for testing
  Serial.begin(115200);
  //srvmtr.attach(SERVO_PIN); 
  // Define Callback Function
  Ps3.attach(notify);
  // Define On Connection Function
  Ps3.attachOnConnect(onConnect);
  // Emulate console as specific MAC address (change as required)
  Ps3.begin("00:00:00:00:00:00");//11:22:33:44:55:6600:00:00:00:00:00
/////////////////////
  Serial.begin(115200);
  Serial.println("Initialization Started");

  // Initialize the pins
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(motorPin1_m3, OUTPUT);
  pinMode(motorPin2_m3, OUTPUT);
  pinMode(motorPin1_m3, OUTPUT);
  pinMode(motorPin2_m3, OUTPUT);
  pinMode(vpPin, INPUT);     // Set the VP pin as an input

  Serial.println("Initialization Completed");

}

void loop() {
  if (!Ps3.isConnected())
    return;
  delay(2000);
///////////////////////////////////

  voltage = analogRead(vpPin) * tune_A * (4.85/4095.0) *((R1+R2)/R2);
  Serial.println(voltage);

}
