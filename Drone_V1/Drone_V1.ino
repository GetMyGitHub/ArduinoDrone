
// INCLUSIONS
#include <Servo.h>
#include <Wire.h>
#include <TimerOne.h>


// MPU9250 DEFINITIONS
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18



// MPU9250 FUNCTIONS
// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


// MOTORS DEFINITIONS
Servo left;
Servo right;
int motorSpeed = 1070;
int comp_right = 3;
int comp_left = 0;

// PID DEFINITION + STABILIZATION // Axe X

boolean endControl = false;

double Kp = 1.7;
double Ki = 0.03;
double Kd = 15;

double x_consigne = 0;

double x1_mesure = 0;
double x2_mesure = 0;

double x1_last_error = 0;
double x2_last_error = 0;

double x1_stab = 0; // motor 1 / axe x / stabilization
double x2_stab = 0; // motor 2 / axe x / stabilization

double x1_stab_error = -(x_consigne - x1_mesure);
double x2_stab_error = (x_consigne - x2_mesure);

double x1_stab_sum_errors = x1_stab_error;
double x2_stab_sum_errors = x2_stab_error;

double x1_stab_variation = x1_stab_error - x1_last_error;
double x2_stab_variation = x2_stab_error - x2_last_error;

// SYSTEM DEFINITIONS
boolean startDrone = false;

void setup() {

  // MOTORS SETUP
  left.attach(9);
  right.attach(6);

  // MPU9250 SETUP
  Wire.begin();
//  // Set accelerometers low pass filter at 5Hz
//  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // Set accelerometers low pass filter at 10Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x04);
//  // Set gyroscope low pass filter at 5Hz
//  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);
  // Set gyroscope low pass filter at 10Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x04);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_2_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

  // SERIAL SETUP
  Serial.begin(115200);

  // OUTPUT BEGIN
  Serial.println("Drone ready : enter 0 to initialize");

}

void loop() {

  if (startDrone) {

    // MPU9250 READER
    // Read accelerometer and gyroscope
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
    // read x gyro value
    int16_t gx = -(Buf[2] << 8 | Buf[3]); // x gyro
    // convert x gyro value in degrees
    float degreeX = (float)gx / 100;
//    Serial.print ("x degree = ");
//    Serial.print (degreeX);
//    Serial.print("\t");

    // PID CALCUL
    
    x1_mesure = degreeX;
    x2_mesure = - degreeX;
    x1_stab_error = (x_consigne - x1_mesure);
//    Serial.print("angle error = ");
//    Serial.println(x1_stab_error);
    
    x2_stab_error = (x_consigne - x2_mesure);
    x1_stab_sum_errors += x1_stab_error;
    x2_stab_sum_errors += x2_stab_error;
    x1_stab_variation = x1_stab_error - x1_last_error;
    x2_stab_variation = x2_stab_error - x2_last_error;
    x1_stab = Kp * x1_stab_error + Ki * x1_stab_sum_errors + Kd * x1_stab_variation;
    x2_stab = Kp * x2_stab_error + Ki * x2_stab_sum_errors + Kd * x2_stab_variation;


    int left_motor_command = motorSpeed + comp_left + (int) x1_stab;
    int right_motor_command = motorSpeed + comp_right + (int) x2_stab;

    if (left_motor_command < motorSpeed) {
      left_motor_command = motorSpeed;
    }

    if (left_motor_command > 1999) {
      left_motor_command = 2000;
    }
    
    if (right_motor_command < motorSpeed) {
      right_motor_command = motorSpeed;
    }

    if (right_motor_command > 1999) {
      right_motor_command = 2000;
    }
    
    x1_last_error = x1_stab_error;
    x2_last_error = x2_stab_error;    

    left.writeMicroseconds(left_motor_command);
    right.writeMicroseconds(right_motor_command);

  }

  // USER CONTROL
  if (Serial.available() > 0) {
    int response = Serial.read();
    if (response == 49) {
      Serial.println("start");
      startDrone = true;
    }
    if (response == 48) {
      Serial.println("init");
      startDrone = false;
      left.writeMicroseconds(1000);
      right.writeMicroseconds(1000);
    }
    if (response == 50) {
      Serial.println("max Throtle");
      //left.writeMicroseconds(2000);
      //right.writeMicroseconds(2000);
    }
    if (response == 43) {
      motorSpeed += 10;
    }
    if (response == 45) {
      motorSpeed -= 10;
    }
  }

  delay(18);
  

}
