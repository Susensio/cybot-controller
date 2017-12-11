#include <Wire.h>


// H-Bridge motor pins                      7 pin socket driver
const byte MOTOR_RIGHT_FORWARDS_PIN = 11;   // 1. Blue
const byte MOTOR_RIGHT_BACKWARDS_PIN = 10;  // 2. Grey
const byte MOTOR_LEFT_FORWARDS_PIN = 9;     // 3. Green
const byte MOTOR_LEFT_BACKWARDS_PIN = 6;    // 4. White

// LED Antennas
const int LED_LEFT_PIN = 2;
const int LED_RIGHT_PIN = 3;

boolean led_left = false;
boolean led_right = false;

byte cmd;

void setup() {
  Serial.begin(9600);
  
  pinMode(MOTOR_RIGHT_FORWARDS_PIN,OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARDS_PIN,OUTPUT);
  pinMode(MOTOR_LEFT_FORWARDS_PIN,OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARDS_PIN,OUTPUT);

  pinMode(LED_LEFT_PIN,OUTPUT);
  pinMode(LED_RIGHT_PIN,OUTPUT);

  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(readI2C); // register event
  
  Serial.println("Hola");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c){
      case 'a': digitalWrite(LED_LEFT_PIN,LOW); break;
      case 'q': digitalWrite(LED_LEFT_PIN,HIGH); break;
      case 'd': digitalWrite(LED_RIGHT_PIN,LOW); break;
      case 'e': digitalWrite(LED_RIGHT_PIN,HIGH); break;
      default: break;
    }
  }
  
  digitalWrite(LED_LEFT_PIN,led_left);
  digitalWrite(LED_RIGHT_PIN,led_right);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void readI2C(int howMany) {
  Serial.print(__FUNCTION__);
  cmd = Wire.read();
  if (Wire.available()){  // More data
    char result[howMany-1];
    int i = 0;
    while (Wire.available()) {
      result[i] = Wire.read();
      i++;
    }
    switch (cmd){
      case 'L': updateLed(result); break;
      default: break;
    }
  }
}

void updateLed(char result[]){
  Serial.print(__FUNCTION__);
  Serial.println(result);
  switch (result[0]){
    case '0': led_left = LOW; break;
    case '1': led_left = HIGH; break;
    case '2': led_right = LOW; break;
    case '3': led_right = HIGH; break;
    default: break;
  }
}

