#include <Wire.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */



const byte INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

volatile uint8_t fifoBuffer[14]; // FIFO storage buffer

// orientation/motion vars
int quaternion[8];      // [w, x, y, z]         quaternion container

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
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
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
        Serial.println(quaternion[0]);
        Serial.println(quaternion[1]);
        Serial.println(quaternion[2]);
        Serial.println(quaternion[3]);
        Serial.println();
    }


}


// Interrupt routine
void dmpDataReady() {
    // read a quaternions from FIFO
    mpu.getFIFOBytes(fifoBuffer, 14);
    // clear the buffer and start over to discard the remaining bytes.
    mpu.resetFIFO();
        
}

void readQuaternion() {
    noInterrupts();
    quaternion[0] = ((fifoBuffer[0] << 8) | fifoBuffer[1]);
    quaternion[1] = ((fifoBuffer[4] << 8) | fifoBuffer[5]);
    quaternion[2] = ((fifoBuffer[8] << 8) | fifoBuffer[9]);
    quaternion[3] = ((fifoBuffer[12] << 8) | fifoBuffer[13]);
    interrupts();
}