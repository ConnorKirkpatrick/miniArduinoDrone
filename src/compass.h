#include "Adafruit_HMC5883_U.h"
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void startCompass(){
  Serial.println("Starting Compass");
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("No Compass detected ... Check your wiring!");
    while(1);
  }
}
float getHeading(attitude currentAttitude){
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");


  //correct for tilt before calculating heading
  //exact use of roll and pitch depends on the device orientation on the drone
  float pitch = DegreestoRads(currentAttitude.pitch);
  float roll = DegreestoRads(currentAttitude.roll);
  float yHorizontal = event.magnetic.x * cos(pitch) + event.magnetic.y*sin(roll)*sin(pitch) - event.magnetic.z * cos(roll)*sin(pitch);
  float xHorizontal = event.magnetic.y * cos(roll) + event.magnetic.z*sin(roll);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(yHorizontal, xHorizontal);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.003839;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;
}