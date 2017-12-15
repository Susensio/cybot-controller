#include <Wire.h>

// H-Bridge motor pins                      7 pin socket driver
const byte MOTOR_RIGHT_FORWARDS_PIN = 11;   // 1. Blue
const byte MOTOR_RIGHT_BACKWARDS_PIN = 10;  // 2. Grey
const byte MOTOR_LEFT_FORWARDS_PIN = 9;     // 3. Green
const byte MOTOR_LEFT_BACKWARDS_PIN = 6;    // 4. White// LED Antennas
const int LED_LEFT_PIN = 2;
const int LED_RIGHT_PIN = 3;

// Robot variables
boolean ledLeft = false;
boolean ledRight = false;
int linearVelocity = 0;
int angularVelocity = 0;


void setup() {
  //Serial.begin(9600);
  
  pinMode(MOTOR_RIGHT_FORWARDS_PIN,OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARDS_PIN,OUTPUT);
  pinMode(MOTOR_LEFT_FORWARDS_PIN,OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARDS_PIN,OUTPUT);

  pinMode(LED_LEFT_PIN,OUTPUT);
  pinMode(LED_RIGHT_PIN,OUTPUT);

  Wire.begin(8);           // join i2c bus with address #8
  Wire.onReceive(readI2C); // register event
  
  //Serial.println("Hola");
}

void loop() {
  digitalWrite(LED_LEFT_PIN,ledLeft);
  digitalWrite(LED_RIGHT_PIN,ledRight);
  reference(linearVelocity, angularVelocity);
}


void readI2C(int howMany) {
  char cmd = Wire.read();
  if (Wire.available()){  // More data
    byte result[howMany-1];
    int i = 0;
    while (Wire.available()) {
      result[i] = Wire.read();
      i++;
    }
    switch (cmd){
      case 'L': updateLeds(result[0]); break;
      case 'M': updateMotors(result); break;
      default: break;
    }
  }
}

void updateLeds(byte result){
  ledLeft = result & B00000010;
  ledRight = result & B00000001;
}

void updateMotors(byte result[]){
  linearVelocity = (result[0]<<8) + result[1];
  angularVelocity = (result[2]<<8) + result[3];
}

void reference(int v, int w){
  v = v/80;
  w = w/20;
  int leftMotor=constrain( v-w ,-255,255);
  int rightMotor=constrain( v+w ,-255,255);
  
  motorSpeed(leftMotor,rightMotor);
}


void motorSpeed(int left, int right){
  leftMotorSpeed(left);
  rightMotorSpeed(right);
}

void rightMotorSpeed(int speed){
  if (speed>0){
    digitalWrite(MOTOR_RIGHT_BACKWARDS_PIN,LOW);
    analogWrite(MOTOR_RIGHT_FORWARDS_PIN,speed);
  } else if (speed<0){
    digitalWrite(MOTOR_RIGHT_FORWARDS_PIN,LOW);
    analogWrite(MOTOR_RIGHT_BACKWARDS_PIN,-speed);
  } else{
    digitalWrite(MOTOR_RIGHT_FORWARDS_PIN,LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARDS_PIN,LOW);
  }
}

void leftMotorSpeed(int speed){
  if (speed>0){
    digitalWrite(MOTOR_LEFT_BACKWARDS_PIN,LOW);
    analogWrite(MOTOR_LEFT_FORWARDS_PIN,speed);
  } else if (speed<0){
    digitalWrite(MOTOR_LEFT_FORWARDS_PIN,LOW);
    analogWrite(MOTOR_LEFT_BACKWARDS_PIN,-speed);
  } else{
    digitalWrite(MOTOR_LEFT_FORWARDS_PIN,LOW);
    digitalWrite(MOTOR_LEFT_BACKWARDS_PIN,LOW);
  }
}
