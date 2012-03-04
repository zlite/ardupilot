/*
  Example of MPU6000 library for ArduIMU v3.
  Code by Jordi Muñoz and Jose Julio. Arducopter development team. DIYDrones.com
*/

#include <SPI.h>
#include <MPU6000.h> // APM 2.0 MPU6000 Library

#define ToDeg(x) (x*57.2957795131)  // radians to degrees :  *180/pi

MPU6000_Class MPU6000;

long timer;
long timer_old;

int8_t MPU6000_arduimuv3_rotation[9] = {1,-2,-3,-4,5,6,-7,8,9};  // rotation for ArduIMU v3

// Print RAW gyro and accel data from sensor registers
void Print_raw_data()
{
  Serial.print("A: ");
  Serial.print ("\t");
  Serial.print(MPU6000.ax());
  Serial.print ("\t");
  Serial.print(MPU6000.ay());
  Serial.print ("\t");
  Serial.print(MPU6000.az());
  Serial.print ("\t");

  Serial.print("G: ");
  Serial.print ("\t");
  Serial.print(MPU6000.gx());
  Serial.print ("\t");
  Serial.print(MPU6000.gy());
  Serial.print ("\t");
  Serial.print(MPU6000.gz());
}

void Print_euler_angles()
{
  Serial.print(ToDeg(MPU6000.roll()));
  Serial.print ("\t");
  Serial.print(ToDeg(MPU6000.pitch()));
  Serial.print ("\t");
  Serial.print(ToDeg(MPU6000.yaw()));
}

void setup()
{	
  Serial.begin(115200);
  Serial.println();
  Serial.println("MPU6000 library example code");

  // First, we configure MPU6000 for ArduIMU v3
  MPU6000.init(MPU6000_ARDUIMUV3_CHIP_SELECT_PIN,MPU6000_ARDUIMUV3_INTPIN);

  Serial.println("Calibrating gyros and accels (keep board flat and still)...");
  MPU6000.gyro_offset_calibration();
  MPU6000.accel_offset_calibration();
  Serial.println("Calibration done.");

  // Initialize the MPU6000 DMP 
  MPU6000.dmp_init();
  MPU6000.set_dmp_rate(MPU6000_50HZ);  // set the dmp output rate to 50Hz

  MPU6000.read();  // Read Raw sensor data
  Print_raw_data();
  Serial.println();
  delay(1000);

  MPU6000.FIFO_reset();
}


void loop()
{
  if (MPU6000.newdata())   // Wait until we have new data from MPU6000 (INT6 interrupt)
  {
    timer_old = timer;
    timer=millis();

    if(MPU6000.FIFO_ready()){
      MPU6000.FIFO_getPacket();   // get DMP attitude data from FIFO
      MPU6000.read();             // read raw sensor data
      MPU6000.calculate(MPU6000_arduimuv3_rotation);        // calculate euler angles solution
      // we run the gyro bias correction from gravity vector algorithm
      MPU6000.gyro_bias_correction_from_gravity();    
    }

    // Output data
    //Serial.print(timer-timer_old);  // Output timing to verify the 200Hz rate
    //Serial.print ("\t");
    Print_euler_angles();
    Serial.println();
  }
}

