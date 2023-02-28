// Most of the mpu stuff is copied from here:
// https://github.com/ElectronicCats/mpu6050/tree/master/examples/MPU6050_DMP6

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// bluetooth stuff
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32.h>
BLEMIDI_CREATE_INSTANCE("Golden Ball", MIDI);

MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

#define INTERRUPT_PIN 34
#define BLE_LED_PIN 32
#define MPU_LED_PIN 19
#define BUTTON_PIN 33
bool bleLedIsBlinking = true;
unsigned long bleLedTimer = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int buttonState = 0;

// orientation/motion vars
Quaternion q;                    // [w, x, y, z]         quaternion container
VectorFloat gravity;             // [x, y, z]            gravity vector
float ypr[3];                    // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool absYpr = true;              // whether to take the abs value of yaw, pitch and roll

// midi vars
int ccs[3] = {0}; // the three midi cc signals that correspond to yaw, pitch and roll
byte midiCh = 1; // MIDI channel to be used
byte cc = 1; // Lowest MIDI CC to be used

unsigned long timer = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // initialize bluetooth connection
  pinMode(BLE_LED_PIN, OUTPUT);
  digitalWrite(BLE_LED_PIN, LOW);
  BLEMIDI.setHandleConnected(OnConnected);
  BLEMIDI.setHandleDisconnected(OnDisconnected);

  MIDI.begin(midiCh);

  pinMode(MPU_LED_PIN, OUTPUT);
  digitalWrite(MPU_LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // initializeGyro();
}

void loop() {
  bleLedBlink();
  // (re)initialize if button has been pressed
  initializeGyro();
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet


    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

//    Serial.print("ypr\t");
//    Serial.print(ypr[0] * 180 / M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180 / M_PI);
//    Serial.print("\t");
//    Serial.println(ypr[2] * 180 / M_PI);

    // calculate midi ccs
    bool outputExists = false;
    if (absYpr) {
      for (int i = 0; i < 3; i++) {
        int absYprDgr = abs(ypr[i] * 180 / M_PI);
        int squashedYprDgr = absYprDgr;
        if (absYprDgr > 90) {
          squashedYprDgr = 180 - absYprDgr;
        }

        //if (absYprDgr < maxYpr[i]) {
        int newCc = map(squashedYprDgr, 0, 90, 0, 127);
        if (abs(newCc - ccs[i]) > 0) {
          outputExists = true;
          ccs[i] = newCc;
          Serial.print("new cc\t");
          Serial.print(cc + i);
          Serial.print("\t");
          Serial.print(newCc);
          MIDI.sendControlChange(cc + i, ccs[i], midiCh);
        }
        //}
      }
    }
    if (outputExists) {
      Serial.println("");
    }
  }
}

void OnConnected() {
  digitalWrite(BLE_LED_PIN, HIGH);
  bleLedIsBlinking = false;
}

void OnDisconnected() {
  bleLedIsBlinking = true;
}

void bleLedBlink() {
  if (bleLedIsBlinking) {
    if ((millis() - bleLedTimer) > 200) {
      digitalWrite(BLE_LED_PIN, !digitalRead(BLE_LED_PIN));
      bleLedTimer = millis();
    }
  }
}

void initializeGyro() {
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH) {
    digitalWrite(MPU_LED_PIN, LOW);
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
    // where do those offsets come from??
    // how is the calibration related to this?
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
      digitalWrite(MPU_LED_PIN, HIGH);
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }
}
