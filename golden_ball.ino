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

unsigned long timer = 0;
//bool soundOn = false;

byte midiCh = 1; //* MIDI channel to be used
byte cc = 1; //* Lowest MIDI CC to be used
int cState[3] = {0};
int angles[3] = {0};
int oldAngles[3] = {0};

void setup() {
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
  Serial.println("Done!\n");
//  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  mpu.update();
  useAngle();
}
//    if (soundOn) {
//      noteOff(0, 48, 100);
//      MidiUSB.flush();
//    } else {
//      noteOn(0, 48, 100);
//      MidiUSB.flush();
//    }
//    soundOn = !soundOn;
//    controlChange(midiCh, cc, cState); //  (channel, CC number,  CC value)
//    MidiUSB.flush();
//    if (cState < 127) {
//      cState += 2;
//    }
//    delay(1000);

//}
//void noteOn(byte channel, byte pitch, byte velocity) {
//
//  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
//
//  MidiUSB.sendMIDI(noteOn);
//}
//
//void noteOff(byte channel, byte pitch, byte velocity) {
//
//  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
//
//  MidiUSB.sendMIDI(noteOff);
//}
//
//void controlChange(byte channel, byte control, byte value) {
//  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
//  MidiUSB.sendMIDI(event);
//}

void useAngle() {
  if((millis()-timer)>100){ // print data every 100ms
    angles[0] = abs(mpu.getAngleX() + 90);
    if (angles[0] > 180) {
      angles[0] = 360 - angles[0];
    }
    angles[1] = mpu.getAngleY() + 90;
    angles[2] = abs(mpu.getAngleZ());
    Serial.print("X : ");
    Serial.print(angles[0]);
    Serial.print("\tY : ");
    Serial.print(angles[1]);
    Serial.print("\tZ : ");
    Serial.println(angles[2]);

    for (int i = 0; i < 3; i++) {
      if (abs(oldAngles[i] - angles[i]) > 2 ) {
        cState[i] = map(angles[i], 0, 180, 0, 127);
//        controlChange(midiCh, cc+i, cState[i]); //  (channel, CC number,  CC value)
//        MidiUSB.flush();
        MIDI.sendControlChange(cc+i, cState[i], midiCh);
        oldAngles[i] = angles[i];
      }
    }
//    cState = map(yAngle, -90, 90, 0, 127);
//    controlChange(midiCh, cc, cState); //  (channel, CC number,  CC value)
//    MidiUSB.flush();
    timer = millis();  
  }
}
