/*
  ESP32 PS3 Controller - LED Control Test
  esp32-ps3-led.ino
  Requires ESP32-PS3 Library - https://github.com/jvpernis/esp32-ps3
  DroneBot Workshop 2023
  https://dronebotworkshop.com
*/
//Wifi libraries 
#include "arduino_secrets.h"
#include "thingProperties.h"

//Controller library
#include <Ps3Controller.h>

//Brushless library
#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager

int Brushless_pin = 23;
Servo ESC;

// setting PWM properties
const int freq = 20000;
const int resolution = 8; 
int motorSpeed = 0;

// Joysticks
int leftY;
int rightx;

// Pin definitions
const int buzzerPin = 26; // Pin where the buzzer is connected
const int ledPin = 27;   // Pin where the LED is connected

//const int motorPin1_m1 = 23; // Weapon
//const int motorPin2_m1 = 22; // Weapon

const int motorPin1_m2 = 21; 
const int motorPin2_m2 = 19; 

const int motorPin1_m3 = 18; 
const int motorPin2_m3 = 17;

// Voltage divider
float voltage;
float R1 = 56000;
float R2 = 18000;
float tune_A = 1.009; // Tuning parameter 
const int vpPin = 36;  // ADC pin

// voltage under 11.7
unsigned long voltageDropStartTime = 0;  // Stores the time when voltage first drops below 11.7V
bool voltageBelowThreshold = false;

const int buzzerFrequency = 2000; // Frequency in Hz
const int buzzerDuration = 500;   // Duration in ms
// Callback Function
void notify() {
   // Get Joystick value
  leftY  = (Ps3.data.analog.stick.ly);
  rightx = (Ps3.data.analog.stick.rx);

// Map the values from -128 to 128 into the range of 0 to 255
  int mappedLeftY  = map(leftY, -128, 128, 257, -257);
  int mappedRightx = map(rightx, -128, 128, 257, -257);
  mappedLeftY  = constrain(mappedLeftY, -255, 255);
  mappedRightx = constrain(mappedRightx, -255, 255);

  /*  FOR TESTING
  Serial.print(" - leftY = ");
  Serial.println(mappedLeftY);
  Serial.print(" - right = ");
  Serial.println(mappedRightx);
*/

if (mappedLeftY > 80) {
  digitalWrite(motorPin1_m2, LOW);
  digitalWrite(motorPin2_m2, HIGH);

  digitalWrite(motorPin2_m3, LOW);
  digitalWrite(motorPin1_m3, HIGH);


} else if (mappedLeftY < -80) {
  digitalWrite(motorPin2_m2, LOW);
  digitalWrite(motorPin1_m2, HIGH);

  digitalWrite(motorPin1_m3, LOW);
  digitalWrite(motorPin2_m3, HIGH);

}

if (mappedRightx > 80) {
  digitalWrite(motorPin2_m2, LOW);
  digitalWrite(motorPin1_m2, HIGH);

  digitalWrite(motorPin2_m3, LOW);
  digitalWrite(motorPin1_m3, HIGH);

} else if (mappedRightx < -80) {
  digitalWrite(motorPin1_m2, LOW);
  digitalWrite(motorPin2_m2, HIGH);

  digitalWrite(motorPin1_m3, LOW);
  digitalWrite(motorPin2_m3, HIGH);
}

// Add this block to stop all motors if both joystick values are within the neutral range
if (mappedLeftY >= -80 && mappedLeftY <= 80 && mappedRightx >= -80 && mappedRightx <= 80) {
  // Stop motor m2
  digitalWrite(motorPin1_m2, LOW);
  digitalWrite(motorPin2_m2, LOW);
  // Stop motor m3
  digitalWrite(motorPin1_m3, LOW);
  digitalWrite(motorPin2_m3, LOW);
}


//////////////////////// WEAPON ///////////////////////////
// Weapon start
    if (Ps3.event.button_down.cross) {
     motorSpeed = 100;
     ESC.write(motorSpeed);
     weapon_PWM = motorSpeed; //wifi

  }
// Weapon start max
    if (Ps3.event.button_down.circle) { // spinning upwards
    motorSpeed = 60;
    ESC.write(motorSpeed);
    weapon_PWM = motorSpeed; //wifi
  }

// Weapon stop
  if (Ps3.event.button_down.square) {
    motorSpeed = 0;
    ESC.write(motorSpeed);
    weapon_PWM = motorSpeed; //wifi
  }
// Weapon stop
  if (Ps3.event.button_down.triangle) {
    motorSpeed = 0;
    ESC.write(motorSpeed);
    weapon_PWM = motorSpeed; //wifi
  }
////////////////////////WEAPON


}

// On Connection function
void onConnect() { 
}


void setup() {
  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  delay(100);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  delay(300);

  // Define Callback Function
  Ps3.attach(notify);
  // Define On Connection Function
  Ps3.attachOnConnect(onConnect);
  // Emulate console as specific MAC address (change as required)
  Ps3.begin("00:00:00:00:00:00"); 

  // Initialize the pins
  pinMode(buzzerPin, OUTPUT); 
  pinMode(ledPin, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(motorPin1_m3, OUTPUT);
  pinMode(motorPin2_m3, OUTPUT);
  pinMode(vpPin, INPUT);     // Set the VP pin as an input 


    ESC.attach(Brushless_pin, 1000, 2000);  // Attach with pulse width for ESC range

    Serial.println("Arming ESC...");
    ESC.write(0);     // Set throttle to zero
    delay(3000);      // Wait for ESC arming beep

    Serial.println("Starting motor...");
    ESC.write(30);    // Set a mid-throttle position
    delay(5000);      // Keep motor running for observation

    ESC.write(0);     // Stop motor
    Serial.println("Stopping motor.");
    
    delay(100);
    tone(buzzerPin, buzzerFrequency, buzzerDuration);
  
}

void loop() {
  ArduinoCloud.update(); // sending values to wifi-display
  if (!Ps3.isConnected())
    ESC.write(0);
    return;
  delay(200); // wass 2000

 /// STOP EVRERYTHING WHEN BATTERY UNDER 11.7V /// /// ///
  voltage = analogRead(vpPin) * tune_A * (3.3/4095.0) *((R1+R2)/R2);

  if (voltage < 11.7) {
    if (!voltageBelowThreshold) {
      // Start the timer if this is the first time voltage drops below the threshold
      voltageDropStartTime = millis();
      voltageBelowThreshold = true;
    }
    
    // Check if voltage has been below threshold for 2 seconds
    if (millis() - voltageDropStartTime >= 4000) {
      // Stop motors if the voltage has been below 11.7V for at least 2 seconds
     // ledcWrite(motorPin2_m1, 0); // Stop weapon
     // ledcWrite(motorPin1_m1, 0);
      ESC.write(0);     // Set throttle to zero

      digitalWrite(motorPin1_m2, LOW); // Stop motor 2
      digitalWrite(motorPin2_m2, LOW);
      
      digitalWrite(motorPin1_m3, LOW); // Stop motor 3
      digitalWrite(motorPin2_m3, LOW);
    }
  } else {
    // Reset the timer and flag if voltage goes above 11.7V
    voltageBelowThreshold = false;
  }
  /// STOP EVRERYTHING WHEN BATTERY UNDER 11.7V /// /// ///


  voltage_12V_Battery = voltage; // sent to wifi-display
  //Serial.println(voltage); // for tuning

}
