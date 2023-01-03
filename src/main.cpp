#include "Arduino.h"
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter


#include "setupGyro.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine
float lateralOffset = 0;
float directionalOffset = 0;
void setup() {
  Serial.begin(115200);
  delay(1000);// Wait for sensor to stabilize
  startGyro();

  ///Set kalman and gyro starting angle
  getData(0);
  kalmanX.setAngle(currentAttitude.pitch); // Set starting angle
  kalmanY.setAngle(currentAttitude.roll);
  gyroXangle = currentAttitude.pitch;
  gyroYangle = currentAttitude.roll;
  timer = micros();
}

void loop() {
  /* Update all the values */
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  currentAttitude = getData(0);

  ///Kalman filter for pitch
  if ((currentAttitude.pitch < -90 && kalAngleX > 90) || (currentAttitude.pitch > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(currentAttitude.pitch);
    kalAngleX = currentAttitude.pitch;
    gyroXangle = currentAttitude.pitch;
  } else {
    kalAngleX = kalmanX.getAngle(
        currentAttitude.pitch, currentAttitude.RAccY, dt); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90)
    currentAttitude.RAccY = -currentAttitude.RAccY; // Invert rate, so it fits the restricted accelerometer reading

  ///Kalman filter for Roll
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((currentAttitude.roll < -90 && kalAngleY > 90) || (currentAttitude.roll > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(currentAttitude.roll);
    kalAngleY = currentAttitude.roll;
    gyroYangle = currentAttitude.roll;
  } else{
    kalAngleY = kalmanY.getAngle(currentAttitude.roll, currentAttitude.RAccX, dt); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90)
    currentAttitude.RAccX = -currentAttitude.RAccX; // Invert rate, so it fits the restricted accelerometer reading

  /*lateralOffset = lateralOffset + (currentAttitude.AccX*dt);
  directionalOffset = directionalOffset + (currentAttitude.AccY*dt);
  Serial.print("LateralOffset:");
  Serial.print(lateralOffset);
  Serial.print(",");
  Serial.print("DirectionalOffset:");
  Serial.print(directionalOffset);
   */

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
    Serial.print("gyroPitch:");
    Serial.print(currentAttitude.gyroPitch);
    Serial.print(",");
    Serial.print("gyroRoll:");
    Serial.print(currentAttitude.gyroRoll);
    Serial.print(",");
    Serial.print("gyroYaw:");
    Serial.print(currentAttitude.gyroYaw);
    */
  Serial.println("");
  delay(50);

}
