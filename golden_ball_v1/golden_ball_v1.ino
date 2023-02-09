/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
//#include "MIDIUSB.h" // apparently the order is important here. There were issues when MIDIUSB was imported before MPU6050_light
//#define LED_BUILTIN 2
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32.h>
BLEMIDI_CREATE_INSTANCE("golden-ball",MIDI);

MPU6050 mpu(Wire);

const byte led_gpio = 32;
unsigned long timer = 0;
//bool soundOn = false;

byte midiCh = 1; //* MIDI channel to be used
byte cc = 1; //* Lowest MIDI CC to be used
int cState[3] = {0};
int angles[3] = {0};
int oldAngles[3] = {0};

void setup() {
  pinMode(led_gpio, OUTPUT);
  digitalWrite(led_gpio, LOW);
  MIDI.begin(midiCh);
//  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  // mpu.setFilterGyroCoef(0.9);
  Serial.println("Done!\n");
  digitalWrite(led_gpio, HIGH);
//  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  mpu.update();
  useAngle();
}

void useAngle() {
  if((millis()-timer)>10){ // print data every 10ms
    angles[0] = abs(mpu.getAngleX() + 90);
    if (angles[0] > 180) {
      angles[0] = 360 - angles[0];
    }
    angles[1] = mpu.getAngleY() + 90;
    angles[2] = int(abs(mpu.getAngleZ())) % 360;
    if (angles[2] > 180) {
      angles[2] = 360 - angles[2];
    }
    Serial.print("X : ");
    Serial.print(angles[0]);
    Serial.print("\tY : ");
    Serial.print(angles[1]);
    Serial.print("\tZ : ");
    Serial.println(angles[2]);

    for (int i = 0; i < 3; i++) {
      if (abs(oldAngles[i] - angles[i]) > 0 ) {
        cState[i] = map(angles[i], 0, 180, 0, 127);
        MIDI.sendControlChange(cc+i, cState[i], midiCh);
        oldAngles[i] = angles[i];
      }
    }
    timer = millis();  
  }
}
