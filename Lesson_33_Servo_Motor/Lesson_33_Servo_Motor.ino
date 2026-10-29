#include <Servo.h>

// Deklarasi objek servo
Servo myservoA;
Servo myservoB;
Servo myservoC;

// Variabel posisi
int pos = 0;

// Deklasi pin servo
const byte servoAPin = 9;
const byte servoBPin = 10;
const byte servoCPin = 11;


void setup() {
  // Inisialisasi pin servo
  myservoA.attach(servoAPin);
  myservoB.attach(servoBPin);
  myservoC.attach(servoCPin);
}

void loop() {
  // Gerakkan servo A, B, C dari sudut 0-180 derajat
  for (pos = 0; pos <= 180; pos += 1) {
    myservoA.write(pos);
    myservoB.write(pos);
    myservoC.write(pos);
    delay(15);
  }

  // Gerakkan servo A, B, C dari 180-0 derajat
  for (pos = 180; pos >= 0; pos -= 1) {
    myservoA.write(pos);
    myservoB.write(pos);
    myservoC.write(pos);
    delay(15);
  }
}