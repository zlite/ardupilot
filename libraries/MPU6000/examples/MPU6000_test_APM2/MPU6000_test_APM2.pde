/*
  Example of MPU6000 library for APM 2.0.
  Code by Jordi Muñoz and Jose Julio. Arducopter development team. DIYDrones.com
*/

#include <SPI.h>
#include <MPU6000.h> // APM 2.0 MPU6000 Library

#define ToDeg(x) (x*57.2957795131)  // radians to degrees :  *180/pi

#define MS5611_CS_pin 40    // MS5611 barometer share the SPI bus on APM2 hardware

MPU6000_Class MPU6000;

long timer;
long timer_old;

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

  // We disable the CS pin on MS5611 to avoid conflicts on SPI bus
  pinMode(MS5611_CS_pin, OUTPUT);
  digitalWrite(MS5611_CS_pin,HIGH);
  
  // First, we configure MPU6000, default settings: 200Hz, 2000dps, 2g, DLPF at 42Hz
  MPU6000.init();

  Serial.println("Calibrating gyros and accels (keep board flat and still)...");
  MPU6000.gyro_offset_calibration();
  MPU6000.accel_offset_calibration();
  Serial.println("Calibration done.");

  // Initialize the MPU6000 DMP internal processor
  MPU6000.dmp_init();
  MPU6000.set_dmp_rate(MPU6000_50HZ);  // set the dmp output data rate at 50Hz (internally runs at 200Hz)

  MPU6000.read();  // Read Raw sensor data
  Print_raw_data();
  Serial.println();
  delay(1000);

  MPU6000.FIFO_reset();   // Reset the FIFO buffer
}


void loop()
{
  if (MPU6000.newdata())   // Wait until we have new data from MPU6000 (INT6 interrupt)
  {
    timer_old = timer;
    timer=millis();

    if(MPU6000.FIFO_ready()){     // new data ready?
      MPU6000.FIFO_getPacket();   // get DMP attitude data from FIFO
      MPU6000.read();             // read raw sensor data
      MPU6000.calculate();        // calculate euler angles solution
      // we run the gyro bias correction using gravity vector algorithm
      MPU6000.gyro_bias_correction_from_gravity();    
    }

    // Output data
    //Serial.print(timer-timer_old);  // Output timing to verify the 200Hz rate
    //Serial.print ("\t");
    Print_euler_angles();
    Serial.println();
  }
}

