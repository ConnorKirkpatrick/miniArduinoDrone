#include "Arduino.h"
#include "Kalman.h"
#include "Adafruit_MotorShield.h"


#include "gyro.h"
#include "barometer.h"
#include "compass.h"
#include "kalmanFilters.h"


/* IMU Data */
uint32_t timer;


float dist = 0;
float distR = 0;
float speed = 0;
float speedR = 0;
// TODO: Make calibration routine
float lateralOffset = 0;
float directionalOffset = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
void setup() {
  Serial.begin(115200);
  //
  /*
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
                       // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  myMotor->setSpeed(10);
  myMotor->run(FORWARD);
  delay(1000);
  myMotor->setSpeed(0);
  /*
  ///P is 40, R is 75, expecting about 3m/s and 6m/s
  double A1 = acos(cos(DegreestoRads(75))*cos(DegreestoRads(90)));
  double A2 = atan((cos(DegreestoRads(75)) * sin(DegreestoRads(90)))/(sin(DegreestoRads(75))));
  Serial.print("Adjust:");
  Serial.print(A1,7);
  Serial.print(" Adjust:");
  Serial.print(A2,7);
  //A1 and A2 bring the vector back to the baseline axis, correcting for any rotation. From here we calculate the vertical component
  Serial.print(" Check:");
  Serial.print(sin(0.5*PI - cos(A1))*9.81,7);

  //A1 and A2 bring the vector back to the base line axis, correcting for any rotation. thus
  Serial.print(" Check:");
  Serial.println(9.81 - sin(0.5*PI - cos(A2))*9.81,7);
  /*
  double pOffset = cos(DegreestoRads(90-A1)) * 9.81;
  double rOffset = cos(DegreestoRads(90-A2)) * 9.81;
  Serial.print("Ang:");
  Serial.print(currentAttitude.pitch);
  Serial.print(" Offset:");
  Serial.print(rOffset);
  Serial.print(" Raw:");
  Serial.print(currentAttitude.AccX)
  */

  delay(1000);// Wait for sensor to stabilize
  Serial.println("SYSTEM STARTED");
  //startBMP();
  startGyro();
  //startCompass();
  ///Set kalman and gyro starting angle
  getGyroData(false);
  setupFilters(currentAttitude);
  timer = micros();


}

void loop() {
  /*
  ///Update all the values
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  currentAttitude = getGyroData(0);
  kalmanUpdate(currentAttitude, dt);
  speedR = speedR + currentAttitude.AccX * dt;
  speed = speed + kalAccX * dt;
  distR = distR + speedR * dt;
  dist = dist + speed * dt;
  /*
    Serial.print("RawPitch:");
    Serial.print(currentAttitude.pitch);
    Serial.print(",");
    Serial.print("KPitch:");
    Serial.print(kalAngleX);
    Serial.print(",");
    Serial.print("RawRoll:");
    Serial.print(currentAttitude.roll);
    Serial.print(",");
    Serial.print("KRoll:");
    Serial.print(kalAngleY);
  */
  /*
  Serial.print("RawACCl:");
  Serial.print(currentAttitude.AccX);
  Serial.print(",");
  Serial.print("KAccl:");
  Serial.print(kalAccX);
  Serial.print(",");
   */

  Serial.println("");
  currentAttitude = getGyroData(0);
  //getHeading(currentAttitude);
  delay(5000);

}
