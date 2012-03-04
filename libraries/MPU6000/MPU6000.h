#ifndef __MPU6000_H__
#define __MPU6000_H__

#include <SPI.h>

#define RAD2DEG 57.295779513

#define MPU6000_APM2_CHIP_SELECT_PIN 53      // MPU6000 CHIP SELECT ON APM 2.0
#define MPU6000_ARDUIMUV3_CHIP_SELECT_PIN 4  // MPU6000 CHIP SELECT ON ARDUIMU V3

#define MPU6000_APM2_INTPIN 6         // MPU6000 connected to INT6 pin
#define MPU6000_ARDUIMUV3_INTPIN 0    // MPU6000 connected to INT0 pin

#define FIFO_PACKET_SIZE 18            // Default quaternion FIFO size (4*4) + Footer(2)
#define MPU_FIFO_BUFFER_SIZE 72        // FIFO buffer size
#define GYRO_BIAS_FROM_GRAVITY_RATE 8  // Rate of the gyro bias from gravity correction (200Hz/8) => 25Hz

#define ACCEL_SCALE_G 8192             // (2G range) G = 8192

// MPU6000 sensibility   (0.0609 => 1/16.4LSB/deg/s at 2000deg/s)
#define Gyro_Gain 0.0609
#define Gyro_Scaled(x) x*Gyro_Gain*0.01745329252 //Return the scaled gyro raw data in radians per second
#define Accel_Scale(x) x*(ACCEL_SCALE_G/9.81)    //Scaling accel units (m/s2) to adc units

#define COMPASS_NULL -9999             // Initial value to detect that compass correction is not initialized

// MPU6000 registers (from Invensense documentation)
#define MPUREG_XG_OFFS_TC 0x00
#define MPUREG_YG_OFFS_TC 0x01
#define MPUREG_ZG_OFFS_TC 0x02
#define MPUREG_X_FINE_GAIN 0x03
#define MPUREG_Y_FINE_GAIN 0x04
#define MPUREG_Z_FINE_GAIN 0x05
#define MPUREG_XA_OFFS_H 0x06
#define MPUREG_XA_OFFS_L 0x07
#define MPUREG_YA_OFFS_H 0x08
#define MPUREG_YA_OFFS_L 0x09
#define MPUREG_ZA_OFFS_H 0x0A
#define MPUREG_ZA_OFFS_L 0x0B
#define MPUREG_PRODUCT_ID 0x0C
#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_XG_OFFS_USRL 0x14
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_YG_OFFS_USRL 0x16
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_ZG_OFFS_USRL 0x18
#define	MPUREG_SMPLRT_DIV 0x19
#define MPUREG_CONFIG 0x1A
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_INT_PIN_CFG 0x37
#define	MPUREG_INT_ENABLE 0x38
#define MPUREG_ACCEL_XOUT_H 0x3B
#define MPUREG_ACCEL_XOUT_L 0x3C
#define MPUREG_ACCEL_YOUT_H 0x3D
#define MPUREG_ACCEL_YOUT_L 0x3E
#define MPUREG_ACCEL_ZOUT_H 0x3F
#define MPUREG_ACCEL_ZOUT_L 0x40
#define MPUREG_TEMP_OUT_H 0x41
#define MPUREG_TEMP_OUT_L 0x42
#define MPUREG_GYRO_XOUT_H 0x43
#define	MPUREG_GYRO_XOUT_L 0x44
#define MPUREG_GYRO_YOUT_H 0x45
#define	MPUREG_GYRO_YOUT_L 0x46
#define MPUREG_GYRO_ZOUT_H 0x47
#define	MPUREG_GYRO_ZOUT_L 0x48
#define MPUREG_USER_CTRL 0x6A
#define	MPUREG_PWR_MGMT_1 0x6B
#define	MPUREG_PWR_MGMT_2 0x6C
#define MPUREG_BANK_SEL 0x6D
#define MPUREG_MEM_START_ADDR 0x6E
#define MPUREG_MEM_R_W 0x6F
#define MPUREG_DMP_CFG_1 0x70
#define MPUREG_DMP_CFG_2 0x71
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_WHOAMI 0x75

// Configuration bits MPU6000
#define BIT_SLEEP 					0x40
#define BIT_H_RESET 				0x80
#define BITS_CLKSEL 				0x07
#define MPU_CLK_SEL_PLLGYROX 		0x01
#define MPU_CLK_SEL_PLLGYROZ 		0x03
#define MPU_EXT_SYNC_GYROX 			0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G					0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define	BIT_INT_ANYRD_2CLEAR	    0x10
#define	BIT_RAW_RDY_EN				0x01
#define	BIT_I2C_IF_DIS              0x10

// DMP output rate constants
#define MPU6000_200HZ 0    // default value
#define MPU6000_100HZ 1
#define MPU6000_66HZ 2
#define MPU6000_50HZ 3


//dmpMem from dmpDefaultMantis.c
extern unsigned char dmpMem[8][16][16] PROGMEM;

// new data flag variable
extern volatile uint8_t _MPU6000_newdata;

// MPU6000 interrupt handler (INT pin)
void MPU6000_newdata_int();

extern int8_t MPU6000_apm2_rotation[9];

// ----- MPU6000 CLASS ----------
class MPU6000_Class
{
  public:

	MPU6000_Class();       // Constructor

	void init();           // MPU6000 initialization  (default cs_pin for APM2.0)
	void init(uint8_t cs_pin, uint8_t int_pin);  // MPU6000 initialization
	void dmp_init();       // MPU6000 DMP initialization
    void set_dmp_rate(uint8_t rate);  // set DMP output rate (see constants)
	
	void read();           // read raw data
	uint8_t newdata();     // new attitude data?
	
	void calculate();      // Read quaternion data and calculate DCM and euler angles (default orientation for APM2.0)
	void calculate(int8_t rotation[9]);  // Same as above but we use an alternative rotation (usefull for other mounting positions, ex: arduimu v3)
	
	// Get methods to access raw sensor data (execute read() method)
	int gx();				// gyroX
	int gy();				// gyroY
	int gz();				// gyroZ
	int ax();				// accelX
	int ay();				// accelY
	int az();				// accelZ
	
	// Attitute output (in radians)
	float roll();
	float pitch();
	float yaw();
	void get_quaternion(float *q);   // get internal quaternion solution q[4]
	void get_DCM(float DCM_matrix[3][3]);        // get DCM matrix in APM2 reference axis DCM[3][3]

	// Calibration methods
	void gyro_offset_calibration();                   // Performs a gyro offset calibration and store offset values
	void accel_offset_calibration();   // Performs an accel offset calibration and store offset values
	void accel_set_offset(int accel_x_offset, int accel_y_offset, int accel_z_offset);  // Set accelerometer offsets
	void accel_get_offset(int *accel_offset);

	// Gyro bias correction methods
	void gyro_bias_correction_from_gravity();             // Function to correct the gyroX and gyroY bias (roll and pitch) using the gravity vector from accelerometers

    // Method to compensate for centrifugal force
    void accel_centrifugal_force_correction(float speed);  // We need to provide and external speed estimation in (m/s)

	// Methods to support external magnetometer fusion
	float compass_angle(float mag_x, float mag_y, float mag_z, float declination);      // Calculate tilt compensated compass heading (from external magnetometer)
	void update_yaw_compass_correction(float compass_heading);                          // Update the yaw correction using compass info (compass sensor fusion)
	void get_yaw_compass_corrected();                                                   // Get the corrected yaw with the compass fusion

    // Functions to retrieve attitude data from FIFO (DMP)
	void FIFO_reset();
    void FIFO_getPacket();
    boolean FIFO_ready();

    // Functions to set gains
    void set_accel_fusion_gain(uint8_t gain);
    void set_gyro_bias_correction_from_gravity_gain(float gain);
    void set_compass_correction_gain(float gain);
    
	private:

	// CS pin
	uint8_t _MPU6000_cs_pin;
	
	//Sensor variables
	int _accelX;
	int _accelY;
	int _accelZ;
    int _MPU6000_ACCEL_OFFSET[3];
	unsigned int _mpu_temp;
	int _gyroX;
	int _gyroY;
	int _gyroZ;

	// Euler angles
	float _MPU_roll;
	float _MPU_pitch;
	float _MPU_yaw;
	float _MPU_yaw_without_compass;

	// quaternion
	float _MPU_q[4];

	// DCM Rotation matrix (in APM2.0 axis definition)
	float _DCM[3][3];
	// internal DCM Rotation matrix (in internal chip axis definition)
	float _DCM_internal[3][3];

	long _accel_filtered[3];
	int  _accel_filtered_samples;
	float _gyro_bias[3]; // bias_tracking
	float _gyro_bias_from_gravity_gain;  // bias correction algorithm gain
	uint8_t _gyro_bias_from_gravity_counter;
	
	// FIFO variables
	uint8_t _received_packet[MPU_FIFO_BUFFER_SIZE];    // FIFO packet buffer
	uint8_t _fifoCountH;
	uint8_t _fifoCountL;
	
	// Compass correction variables
	float _yaw_compass_diff;
	float _compass_correction_gain;
	float _gyro_bias_from_compass_gain;  // default value (not used now)

    // private methods
    uint8_t _SPI_read(uint8_t reg);
    void _SPI_write(uint8_t reg, uint8_t data);
    void _set_mpu_memory(uint8_t bank, uint8_t address, uint8_t num, uint8_t regs[]);
    void _set_dmp_gyro_calibration();
    void _set_dmp_accel_calibration();
    void _apply_endian_accel();
    void _set_mpu_sensors();
    void _set_bias_from_no_motion();
    void _set_bias_none();
    void _set_fifo_interrupt();
    void _send_quaternion();
    void _send_gyro();
    void _send_accel();
    void _set_fifo_rate(uint8_t rate);
    void _set_sensor_fusion_accel_gain(uint8_t gain);
    void _dmp_load_mem();
    float _normalize_angle(float angle);
    void _gyro_offset_update(int offsetX, int offsetY, int offsetZ);
    float _gyro_bias_correction_from_compass(float heading);
};

#endif // __MPU6000_H__
