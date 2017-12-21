#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <TaskScheduler.h>

#include "fxpt_atan2.h"

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.print(x)
 #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

MPU6050 mpu;

const byte INTERRUPT_PIN = 2;  // use pin 2 on Arduino Uno & most boards

volatile uint8_t fifoBuffer[14]; // FIFO storage buffer


unsigned long interval = 100;
unsigned long previousTime;

void control();
Task controlTask(100, TASK_FOREVER, &control);
Scheduler runner;


void control() {
    static int lastYaw;

    int yaw = getYaw();
    DEBUG_PRINTLN(yaw);

    int angularVelocity = yaw - lastYaw;

    DEBUG_PRINTLN(angularVelocity);
    lastYaw = yaw;
}

void setup() {
    // initialize serial communication
    Serial.begin(115200);

    mpuSetup();
    previousTime = millis();
}


void loop() {
    runner.execute();
}


void mpuSetup() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    DEBUG_PRINTLN(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    DEBUG_PRINTLN(F("Testing device connections..."));
    DEBUG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    DEBUG_PRINTLN(F("Initializing DMP..."));
    mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // turn on the DMP, now that it's ready
    DEBUG_PRINTLN(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    DEBUG_PRINTLN(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    DEBUG_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
}

// Interrupt routine
void dmpDataReady() {
    // read a quaternions from FIFO
    mpu.getFIFOBytes(fifoBuffer, 14);
    // clear the buffer and start over to discard the remaining bytes.
    mpu.resetFIFO();
        
}

int getYaw() {
    noInterrupts();
    int w = ((fifoBuffer[0] << 8) | fifoBuffer[1]);
    int x = ((fifoBuffer[4] << 8) | fifoBuffer[5]);
    int y = ((fifoBuffer[8] << 8) | fifoBuffer[9]);
    int z = ((fifoBuffer[12] << 8) | fifoBuffer[13]);
    interrupts();

    DEBUG_PRINTLN(w);
    DEBUG_PRINTLN(x);
    DEBUG_PRINTLN(y);
    DEBUG_PRINTLN(z);
    DEBUG_PRINTLN();

    //atan2(2*x*y - 2*w*z, 2*w*w + 2*x*x - 1);  all divided by 4
    int yaw = (int)fxpt_atan2((q15_mul(x,y)/2 - q15_mul(w,z)/2),
                    (q15_mul(w,w)/2 + (q15_mul(x,x) - 0x4000)/2));

    return yaw;
}
