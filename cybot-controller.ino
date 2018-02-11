#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <SPI.h>

#include <TaskScheduler.h>


//#include "fxpt_atan2.h"

// L298 N Motor driver pins
const byte MOTOR_RIGHT_ENABLE_PIN = 9;     // ENA
const byte MOTOR_RIGHT_FORWARDS_PIN = 8;    // IN1
const byte MOTOR_RIGHT_BACKWARDS_PIN = 7;   // IN2

const byte MOTOR_LEFT_ENABLE_PIN = 6;       // ENB
const byte MOTOR_LEFT_FORWARDS_PIN = 5;     // IN3
const byte MOTOR_LEFT_BACKWARDS_PIN = 4;    // IN4

const int LED_LEFT_PIN = A0;
const int LED_RIGHT_PIN = A1;

// MPU variables
MPU6050 mpu;
volatile uint8_t fifoBuffer[14]; // FIFO storage buffer

// SPI variables
volatile char spiBuffer [10];
volatile byte pos;
volatile byte parityByte;

// Interrupts
const byte I2C_INTERRUPT_PIN = 2;  // use pin 2 on Arduino Uno & most boards
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
const byte SPI_INTERRUPT_PIN = 3;
volatile bool spiInterrupt;
void spiDataReady() {
    spiInterrupt = true;
}
ISR (SPI_STC_vect) {
  spiBuffer [pos++] = SPDR; // grab byte from SPI Data Register
}



// Robot variables
boolean ledLeft = false;
boolean ledRight = false;
int linearVelocityRef = 0;
int angularVelocityRef = 0;

// Control variables
int sampling_time = 100;
int K_regulator = 4;

// Scheduler
void controlCallback();
Task controlTask(sampling_time, TASK_FOREVER, &controlCallback);
Scheduler runner;

Quaternion q;           // [w, x, y, z]         quaternion container

void controlCallback() {
    float yaw = getYaw();
    float angularVelocity = getAngularVelocity(yaw);
    
    //Serial.println (linearVelocityRef);
    //Serial.println (angularVelocityRef);
    setReference(linearVelocityRef, angularVelocityRef + K_regulator*(angularVelocityRef - 1.3*angularVelocity));


//    Serial.print(yaw);
//    Serial.print('\t');
//    Serial.print(angularVelocity);
//    Serial.print('\t');
//    Serial.print(linearVelocityRef);
//    Serial.print('\t');
//    Serial.println(angularVelocityRef);
}

void setup() {
    // initialize serial communication
    Serial.begin(115200);

    spiSetup();
    mpuSetup();
    //ledsSetup();
    
    delay(500);
    motorsSetup();

    delay(1000);
    runner.init();
    runner.addTask(controlTask);
    controlTask.enable();
    
    Serial.println("Holi");
}


void loop() {
    mpuRead();
    spiRead();
    
    runner.execute();
}

void spiRead(){
    if (spiInterrupt) { 
        Serial.println("spiRead.spiInterrupt");
        // Parity check
        byte parityByte = spiBuffer[pos-1];
        Serial.println(parityByte);
        byte parity = 0;
        for (int i=0; i<pos-1; i++){
          Serial.print(spiBuffer[i],HEX);
          parity += spiBuffer[i];
        }
        Serial.println();
        

        // Data received correctly
        if (parityByte == parity){
          Serial.println ('!');
          Serial.println (spiBuffer[0]);
          //linearVelocityRef = 0;
          if (spiBuffer[0] == 'M'){
            linearVelocityRef = (spiBuffer[1]<<8) | spiBuffer[2] & 0xFF;
            angularVelocityRef = (spiBuffer[3]<<8) | spiBuffer[4] & 0xFF;
            
            Serial.println (linearVelocityRef); 
            Serial.println (angularVelocityRef);  
          }
        }

        // Reset Bufffer
        pos = 0;
        spiInterrupt = false;
    }
}
void mpuRead(){
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
}

float getAngularVelocity(float yaw){
    static float lastYaw;

    //float yaw = getYaw();

    // Yaw overflows [-180:180]
    float yawDifference = yaw - lastYaw;
    
    if (yawDifference > 180)
      yawDifference -= 360;
    else if (yawDifference < -180)
      yawDifference += 360;

    float angularVelocity = (yawDifference)*1000/sampling_time;

    lastYaw = yaw;

    return angularVelocity;
}

float getYaw(){
    //noInterrupts();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //interrupts();
    
    float yaw = atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1) * 180/M_PI;
    
    return -yaw;
}

// Reference values in range [-1024:+1024] aprox
void setReference(int v, int w){
    int leftMotorSpeed=constrain( (v-w)/4 ,-255,255);
    int rightMotorSpeed=constrain( (v+w)/4 ,-255,255);
    
//    Serial.print(rightMotorSpeed);
//    Serial.print('\t');
//    Serial.println(leftMotorSpeed);

    setRightMotorSpeed(rightMotorSpeed);
    setLeftMotorSpeed(leftMotorSpeed);
}

void setRightMotorSpeed(int speed){
    if (speed>0){
        digitalWrite(MOTOR_RIGHT_FORWARDS_PIN,HIGH);
        digitalWrite(MOTOR_RIGHT_BACKWARDS_PIN,LOW);
    } else if (speed<0){
        digitalWrite(MOTOR_RIGHT_FORWARDS_PIN,LOW);
        digitalWrite(MOTOR_RIGHT_BACKWARDS_PIN,HIGH);
    } else{
        digitalWrite(MOTOR_RIGHT_FORWARDS_PIN,LOW);
        digitalWrite(MOTOR_RIGHT_BACKWARDS_PIN,LOW);
    }
    analogWrite(MOTOR_RIGHT_ENABLE_PIN, abs(speed));
}

void setLeftMotorSpeed(int speed){
    if (speed>0){
        digitalWrite(MOTOR_LEFT_FORWARDS_PIN,HIGH);
        digitalWrite(MOTOR_LEFT_BACKWARDS_PIN,LOW);
    } else if (speed<0){
        digitalWrite(MOTOR_LEFT_FORWARDS_PIN,LOW);
        digitalWrite(MOTOR_LEFT_BACKWARDS_PIN,HIGH);
    } else{
        digitalWrite(MOTOR_LEFT_FORWARDS_PIN,LOW);
        digitalWrite(MOTOR_LEFT_BACKWARDS_PIN,LOW);
    }
    analogWrite(MOTOR_LEFT_ENABLE_PIN, abs(speed));
}


/************* SETUP ROUTINES *************/

void spiSetup(){
  // turn on SPI in slave mode
  SPCR |= bit (SPE);

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // get ready for an interrupt 
  pos = 0;   // Buffer empty
  spiInterrupt = false;

  // now turn on interrupts
  SPI.attachInterrupt();
  attachInterrupt(digitalPinToInterrupt(SPI_INTERRUPT_PIN), spiDataReady, RISING);
}

void mpuSetup() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(I2C_INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-3060);
    mpu.setYAccelOffset(-1733);
    mpu.setZAccelOffset(1480);
    mpu.setXGyroOffset(82);
    mpu.setYGyroOffset(22);
    mpu.setZGyroOffset(6);

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(I2C_INTERRUPT_PIN), dmpDataReady, RISING);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    
}

void motorsSetup() {
    
    pinMode(MOTOR_RIGHT_ENABLE_PIN,OUTPUT);
    pinMode(MOTOR_LEFT_ENABLE_PIN,OUTPUT);
    delay(500);
    
    pinMode(MOTOR_RIGHT_FORWARDS_PIN,OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARDS_PIN,OUTPUT);

    pinMode(MOTOR_LEFT_FORWARDS_PIN,OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARDS_PIN,OUTPUT);
    
}

void ledsSetup() {
    pinMode(LED_LEFT_PIN,OUTPUT);
    pinMode(LED_RIGHT_PIN,OUTPUT);
}


