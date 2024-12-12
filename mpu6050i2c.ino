//MPU6050 i2c communication using wire.h library with arduino uno

#include "Wire.h"


/*--------------------------------------------------------------- Define Addresses ---------------------------------------------- */
//from the ivensense mpu6050 product specifications, i2c address is 1101000

#define mpu6050_address 0b1101000

//from the ivensense mpu6050 Register Map and Descriptions

//1. power management allows mpu6050 to be woken up from sleep more
  //set bit 6 to 0 to disable sleep mode, 
#define pwr_mgmt_1 0x6b

//2. accelerometer configuration allows us to use the full range of the accelerometer
 //for the application of a pedometer, more accuracy is desired, so set to +-2g by setting bits 3 and 4 to 0
 #define accel_config 0x1c

//3. gyroscope configuration allows us to use full scale range of the gyroscope
  //for the application of a pedometer, more accuracy is desired, so set to +-250 by setting bits 3 and 4 to 0
#define gyro_config 0x1b

//4. accelerometer data is stored in registers 59-64 with the most recent measurements
 //each x, y, z acceleromter measurement is a 16 bit 2's complement 
#define accel_xout 0x3b 
#define accel_yout 0x3d
#define accel_zout 0x3f

//5. gyroscope data is stored in registers 67-72 with the most ecent measurements
  //each x, y, z gyroscope measurement is a 16 buit 2's complement
#define gyro_xout 0x43
#define gyro_yout 0x45
#define gyro_zout 0x47



/*--------------------------------------------------------- Initialization --------------------------------------------------------*/

void setup(){

  //enable internal pull up resistors for sda and scl line
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  //start the serial communication, where values will be displayed
  Serial.begin(115200); 

  //initialize the arduino as the master device by calling begin function with no argument
  Wire.begin();

  //initialize MPU6050 by writing to require registers
    //wake up mpu6050 by setting 6th bit to 0 (0x40 --> 0x00)
  write (pwr_mgmt_1, 0x00);
    //configure mpu6050 acceleromater + gyroscope by setting 3rd and 4th bit to 0 (0x00 --> 0x00)
  write (accel_config, 0x00);
  write (gyro_config, 0x00);

  Serial.println("MPU6050 has been initialized!!");

}

/*--------------------------------------------------------- Read and Print Data --------------------------------------------------------*/

void loop() {
  // Read accelerometer data
  int16_t accelX = read(accel_xout);
  int16_t accelY = read(accel_yout);
  int16_t accelZ = read(accel_zout);

  //read gyroscope date
  int16_t gyroX = read(gyro_xout);
  int16_t gyroY = read(gyro_yout);
  int16_t gyroZ = read(gyro_zout);
  

  // Print accelerometer data
  Serial.print("Accel X: "); Serial.println(accelX);
  Serial.print("Accel Y: "); Serial.println(accelY);
  Serial.print("Accel Z: "); Serial.println(accelZ);

  // Print gyroscope data
  Serial.print("Gyro X: "); Serial.println(gyroX);
  Serial.print("Gyro Y: "); Serial.println(gyroY);
  Serial.print("Gyro Z: "); Serial.println(gyroZ);

  delay(1000);  // Wait 1 second
}



/*------------------------------------- Functions to read/write data to MPU6050 --------------------------------*/

//1. function that will read sensor's signed 16 bit data in x, y, z direction
int16_t read(uint8_t addy){

  //begin transmission from mpu6050 to arduino
  Wire.beginTransmission(mpu6050_address);

  //tell mpu6050 to read the address specified in function call
  Wire.write(addy);

  //
  Wire.endTransmission(false);

  //arduino request 16 bits or 2 bytes from mpu6050 
  Wire.requestFrom(mpu6050_address, 2);

  //check if enough data was received by arduino (master)
    //if less than 2 bytes were received, error code and return 0
  if (Wire.available() < 2) {
    Serial.println("not enough bytes received, check connection!!");
    return 0;
  }

  //store the first read byte which represents data from 15-8
  int8_t msbyte = Wire.read();
  //store the second byte which represents data from 7-0
  int8_t lsbyte = Wire.read();

  //combine stored bytes to create signed 16 bit number
  return (msbyte << 8) | lsbyte;

}

//2. funcion that will write to sensor's register to initialize i2c protocol
void write(uint8_t addy, uint8_t data){
  //begin tranmission from mpu6050 to arduino
  Wire.beginTransmission(mpu6050_address);

  Wire.write(addy); 
  Wire.write(data);

  if (Wire.endTransmission() != 0){
    Serial.println("error writing to mpu6050!");
  }

}


