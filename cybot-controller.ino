#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <TaskScheduler.h>

#include "fxpt_atan2.h"


MPU6050 mpu;

const byte INTERRUPT_PIN = 2;  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

volatile uint8_t fifoBuffer[14]; // FIFO storage buffer


unsigned long interval = 100;
unsigned long previousTime;



void setup() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    previousTime = millis();
}


void loop() {
    // if programming failed, don't try to do anything
    if (devStatus) return;

    if (millis() - previousTime >= interval) {
        previousTime += interval;

        readQuaternion();


        int yaw = getYaw();

        
    }


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

    Serial.println(w);
    Serial.println(x);
    Serial.println(y);
    Serial.println(z);
    Serial.println();

    //atan2(2*x*y - 2*w*z, 2*w*w + 2*x*x - 1);  all divided by 4
    int yaw = (int)fxpt_atan2((q15_mul(x,y)/2 - q15_mul(w,z)/2),
                    (q15_mul(w,w)/2 + (q15_mul(x,x) - 0x4000)/2));

    return yaw;
}
