/* MPU9250_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is intended specifically for the MPU9250+BMP280 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the MPU9250+MS5637 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#include "Wire.h"   
//#include <i2c_t3.h>
//#include <SPI.h>

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define MPU9250_SELF_TEST_X_GYRO 0x00                  
#define MPU9250_SELF_TEST_Y_GYRO 0x01                                                                          
#define MPU9250_SELF_TEST_Z_GYRO 0x02

#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E    
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F

#define MPU9250_SELF_TEST_A      0x10

#define MPU9250_XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define MPU9250_XG_OFFSET_L      0x14
#define MPU9250_YG_OFFSET_H      0x15
#define MPU9250_YG_OFFSET_L      0x16
#define MPU9250_ZG_OFFSET_H      0x17
#define MPU9250_ZG_OFFSET_L      0x18
#define MPU9250_SMPLRT_DIV       0x19
#define MPU9250_CONFIG           0x1A
#define MPU9250_GYRO_CONFIG      0x1B
#define MPU9250_ACCEL_CONFIG     0x1C
#define MPU9250_ACCEL_CONFIG2    0x1D
#define MPU9250_LP_ACCEL_ODR     0x1E   
#define MPU9250_WOM_THR          0x1F   

#define MPU9250_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU9250_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define MPU9250_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define MPU9250_FIFO_EN          0x23
#define MPU9250_I2C_MST_CTRL     0x24   
#define MPU9250_I2C_SLV0_ADDR    0x25
#define MPU9250_I2C_SLV0_REG     0x26
#define MPU9250_I2C_SLV0_CTRL    0x27
#define MPU9250_I2C_SLV1_ADDR    0x28
#define MPU9250_I2C_SLV1_REG     0x29
#define MPU9250_I2C_SLV1_CTRL    0x2A
#define MPU9250_I2C_SLV2_ADDR    0x2B
#define MPU9250_I2C_SLV2_REG     0x2C
#define MPU9250_I2C_SLV2_CTRL    0x2D
#define MPU9250_I2C_SLV3_ADDR    0x2E
#define MPU9250_I2C_SLV3_REG     0x2F
#define MPU9250_I2C_SLV3_CTRL    0x30
#define MPU9250_I2C_SLV4_ADDR    0x31
#define MPU9250_I2C_SLV4_REG     0x32
#define MPU9250_I2C_SLV4_DO      0x33
#define MPU9250_I2C_SLV4_CTRL    0x34
#define MPU9250_I2C_SLV4_DI      0x35
#define MPU9250_I2C_MST_STATUS   0x36
#define MPU9250_INT_PIN_CFG      0x37
#define MPU9250_INT_ENABLE       0x38
#define MPU9250_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define MPU9250_INT_STATUS       0x3A
#define MPU9250_ACCEL_XOUT_H     0x3B
#define MPU9250_ACCEL_XOUT_L     0x3C
#define MPU9250_ACCEL_YOUT_H     0x3D
#define MPU9250_ACCEL_YOUT_L     0x3E
#define MPU9250_ACCEL_ZOUT_H     0x3F
#define MPU9250_ACCEL_ZOUT_L     0x40
#define MPU9250_TEMP_OUT_H       0x41
#define MPU9250_TEMP_OUT_L       0x42
#define MPU9250_GYRO_XOUT_H      0x43
#define MPU9250_GYRO_XOUT_L      0x44
#define MPU9250_GYRO_YOUT_H      0x45
#define MPU9250_GYRO_YOUT_L      0x46
#define MPU9250_GYRO_ZOUT_H      0x47
#define MPU9250_GYRO_ZOUT_L      0x48
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60
#define MPU9250_MOT_DETECT_STATUS 0x61
#define MPU9250_I2C_SLV0_DO      0x63
#define MPU9250_I2C_SLV1_DO      0x64
#define MPU9250_I2C_SLV2_DO      0x65
#define MPU9250_I2C_SLV3_DO      0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET  0x68
#define MPU9250_MOT_DETECT_CTRL  0x69
#define MPU9250_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define MPU9250_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define MPU9250_PWR_MGMT_2       0x6C
#define MPU9250_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define MPU9250_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define MPU9250_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define MPU9250_DMP_REG_1        0x70
#define MPU9250_DMP_REG_2        0x71 
#define MPU9250_FIFO_COUNTH      0x72
#define MPU9250_FIFO_COUNTL      0x73
#define MPU9250_FIFO_R_W         0x74
#define MPU9250_WHO_AM_I         0x75 // Should return 0x71
#define MPU9250_XA_OFFSET_H      0x77
#define MPU9250_XA_OFFSET_L      0x78
#define MPU9250_YA_OFFSET_H      0x7A
#define MPU9250_YA_OFFSET_L      0x7B
#define MPU9250_ZA_OFFSET_H      0x7D
#define MPU9250_ZA_OFFSET_L      0x7E

// BMP280 registers
#define BMP280_TEMP_XLSB  0xFC
#define BMP280_TEMP_LSB   0xFB
#define BMP280_TEMP_MSB   0xFA
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_PRESS_LSB  0xF8
#define BMP280_PRESS_MSB  0xF7
#define BMP280_CONFIG     0xF5
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_STATUS     0xF3
#define BMP280_RESET      0xE0
#define BMP280_ID         0xD0  // should be 0x58
#define BMP280_CALIB00    0x88

#define MPU9250_ADDRESS 0x68       // MPU9250 address when ADO = 1
#define AK8963_ADDRESS 0x0C        // Address of AK8963 (MPU9250) magnetometer
#define BMP280_ADDRESS 0x77        // Address of BMP280 altimeter 

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters

#define sp(x) Serial.print(x)
#define spl(x) Serial.println(x)
#define spf(x) Serial.print(F(x))
#define spfl(x) Serial.println(F(x))

// Pin definitions
int myLed = 13;

float SelfTest[6];            // holds results of gyro and accelerometer self test

void setup()
{

  Wire.begin();
  Serial.begin(115200);
  //delay(1000);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  //delay(1000);
  sp( __DATE__);spf(" : "); spl( __TIME__);

   // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 9-axes motion senseur...");
  byte c = readByte(MPU9250_ADDRESS, MPU9250_WHO_AM_I);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  delay(1000); 
	if (c == 0x71) {// WHO_AM_I should always be 0x68 
		spfl("MPU9250 is online...");
		
		MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
		Serial.println("MPU9250 Self Test:");
		Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
		Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
		Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
		Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
		Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
		Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
		delay(1000);
	} // accolade a virer



    /*
   // get sensor resolutions, only need to do this once
   MPU9250getAres();
   MPU9250getGres();
   MPU9250getMres();
    
   Serial.println(" Calibrate MPU9250 gyro and accel");
   accelgyrocalMPU9250(MPU9250gyroBias, MPU9250accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
   Serial.println("accel biases (mg)"); Serial.println(1000.*MPU9250accelBias[0]); Serial.println(1000.*MPU9250accelBias[1]); Serial.println(1000.*MPU9250accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(MPU9250gyroBias[0]); Serial.println(MPU9250gyroBias[1]); Serial.println(MPU9250gyroBias[2]);

  delay(1000);  
   
  initMPU9250(); 
  Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

  delay(1000); 
  
  // Get magnetometer calibration from AK8963 ROM
  initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
  
  magcalMPU9250(MPU9250magBias);
  Serial.println("AK8963 mag biases (mG)"); Serial.println(MPU9250magBias[0]); Serial.println(MPU9250magBias[1]); Serial.println(MPU9250magBias[2]); 
  delay(2000); // add delay to see results before serial spew of data
   
  if(SerialDebug) {
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
  }
  
  delay(1000);  
  
    }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
  
  // Read the WHO_AM_I register of the BMP280 this is a good test of communication
  byte f = readByte(BMP280_ADDRESS, BMP280_ID);  // Read WHO_AM_I register for BMP280
  Serial.print("BMP280 "); 
  Serial.print("I AM "); 
  Serial.print(f, HEX); 
  Serial.print(" I should be "); 
  Serial.println(0x58, HEX);
  Serial.println(" ");

  delay(1000); 

  writeByte(BMP280_ADDRESS, BMP280_RESET, 0xB6); // reset BMP280 before initilization
  delay(100);

  BMP280Init(); // Initialize BMP280 altimeter
  Serial.println("BMP280 Calibration coeficients:");
  Serial.print("dig_T1 ="); 
  Serial.println(dig_T1);
  Serial.print("dig_T2 ="); 
  Serial.println(dig_T2);
  Serial.print("dig_T3 ="); 
  Serial.println(dig_T3);
  Serial.print("dig_P1 ="); 
  Serial.println(dig_P1);
  Serial.print("dig_P2 ="); 
  Serial.println(dig_P2);
  Serial.print("dig_P3 ="); 
  Serial.println(dig_P3);
  Serial.print("dig_P4 ="); 
  Serial.println(dig_P4);
  Serial.print("dig_P5 ="); 
  Serial.println(dig_P5);
  Serial.print("dig_P6 ="); 
  Serial.println(dig_P6);
  Serial.print("dig_P7 ="); 
  Serial.println(dig_P7);
  Serial.print("dig_P8 ="); 
  Serial.println(dig_P8);
  Serial.print("dig_P9 ="); 
  Serial.println(dig_P9);
 */
}

void loop()
{



}

   uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive

	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	// Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( uint16_t ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);  
   writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }
   
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest){  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	// Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
       Wire.requestFrom(address, count);  // Read bytes from slave register address 
        // Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}
