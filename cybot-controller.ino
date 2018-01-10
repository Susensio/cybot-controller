#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <TaskScheduler.h>

//#include "fxpt_atan2.h"

// H-Bridge motor pins                      7 pin socket driver
const byte MOTOR_RIGHT_FORWARDS_PIN = 11;   // 1. Blue
const byte MOTOR_RIGHT_BACKWARDS_PIN = 10;  // 2. Grey
const byte MOTOR_LEFT_FORWARDS_PIN = 9;     // 3. Green
const byte MOTOR_LEFT_BACKWARDS_PIN = 6;    // 4. White// LED Antennas

const int LED_LEFT_PIN = 2;
const int LED_RIGHT_PIN = 3;

// Interrupt
const byte INTERRUPT_PIN = 2;  // use pin 2 on Arduino Uno & most boards
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// MPU variables
MPU6050 mpu;
volatile uint8_t fifoBuffer[14]; // FIFO storage buffer

// Robot variables
boolean ledLeft = false;
boolean ledRight = false;
int linearVelocityRef = 0;
int angularVelocityRef = 0;

// Scheduler
void controlCallback();
Task controlTask(100, TASK_FOREVER, &controlCallback);
Scheduler runner;



Quaternion q;           // [w, x, y, z]         quaternion container


void controlCallback() {
    static float lastYaw;

    float yaw = getYaw();
    //Serial.println(yaw);

    float angularVelocity = (yaw - lastYaw);

    setReference(linearVelocityRef, 2*angularVelocityRef - angularVelocity);


    Serial.println(angularVelocity);
    lastYaw = yaw;
}

void setup() {
    // initialize serial communication
    Serial.begin(115200);

    motorsSetup();
    ledsSetup();

    mpuSetup();
    
    runner.init();
    runner.addTask(controlTask);
    controlTask.enable();
}


void loop() {
    if (mpuInterrupt) {
        mpuInterrupt = false;
        int mpuIntStatus = mpu.getIntStatus();
        int fifoCount = mpu.getFIFOCount();
        
        if ((mpuIntStatus & 0x10) || fifoCount == 1024)
            // reset so we can continue cleanly
            mpu.resetFIFO();
            
        if (mpuIntStatus & 0x02 && fifoCount >= 16) {
            // read a quaternions from FIFO
            mpu.getFIFOBytes(fifoBuffer, 14);
            // clear the buffer and start over to discard the remaining bytes.
            mpu.resetFIFO();
        }
    }
    
    runner.execute();
}


float getYaw() {
    noInterrupts();
    int w = ((fifoBuffer[0] << 8) | fifoBuffer[1]);
    int x = ((fifoBuffer[4] << 8) | fifoBuffer[5]);
    int y = ((fifoBuffer[8] << 8) | fifoBuffer[9]);
    int z = ((fifoBuffer[12] << 8) | fifoBuffer[13]);
    interrupts();

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    float yaw = atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1) * 180/M_PI;
    
    return yaw;
}

// Reference values in range [-1024:+1024] aprox
void setReference(int v, int w){
 int leftMotorSpeed=constrain( (v-w)/4 ,-255,255);
 int rightMotorSpeed=constrain( (v+w)/4 ,-255,255);
 
 setRightMotorSpeed(rightMotorSpeed);
 setLeftMotorSpeed(leftMotorSpeed);
}

void setRightMotorSpeed(int speed){
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

void setLeftMotorSpeed(int speed){
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


/************* SETUP ROUTINES *************/

void mpuSetup() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setXAccelOffset(5);
    mpu.setYAccelOffset(10);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    
}

void motorsSetup() {
    pinMode(MOTOR_RIGHT_FORWARDS_PIN,OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARDS_PIN,OUTPUT);
    pinMode(MOTOR_LEFT_FORWARDS_PIN,OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARDS_PIN,OUTPUT);
}

void ledsSetup() {
    pinMode(LED_LEFT_PIN,OUTPUT);
    pinMode(LED_RIGHT_PIN,OUTPUT);
}


