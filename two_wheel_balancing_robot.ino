#include <SimpleFOC.h>  // ver. 1.61

BLDCMotor motorL = BLDCMotor(3, 5, 6, 7); 
MagneticSensorAnalog sensorL = MagneticSensorAnalog(A0, 14, 1020);

BLDCMotor motorR = BLDCMotor(9, 10, 11, 7);  
MagneticSensorAnalog sensorR = MagneticSensorAnalog(A1, 14, 1020);

#define MPU6050 0x68              // Device address
#define ACCEL_CONFIG 0x1C         // Accelerometer configuration address
#define GYRO_CONFIG 0x1B          // Gyro configuration address

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0                 // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1                // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

#define    STX          0x02
#define    ETX          0x03
int16_t   joyY, joyX;                           // smartphone joystick input values
uint8_t   cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};    // bytes received
uint8_t   buttonStatus = 0;                     // first Byte sent to Android device
uint8_t   resp_rate = 1;                        

int16_t  AcX, AcZ, GyY, gyroY;

//IMU offset values
int16_t  AcX_offset = -60;
int16_t  AcZ_offset = 300;
int16_t  GyY_offset = 0;
int32_t  GyY_offset_sum = 0;

float robot_angle;
float Acc_angle;            // angle calculated from acc. measurments

bool vertical = false;      
bool test = true;           // motor test

float rot_Speed = 0;
float move_Speed = 0;
float robot_speed = 0, robot_position = 0;

float K1 = 0.44; 
float K2 = 0.036; 
float K3 = 0.25; 
float K4 = 0.20; 

long loop_count = 0, loop2_count = 0;
int loop_time = 5;

#define Gyro_amount 0.996   // percent of gyro in complementary filter

uint32_t timer;

void setup() {

  Serial.begin(115200);
  delay(1000);
  
  sensorL.init();                                                // initialise magnetic sensor hardware
  motorL.linkSensor(&sensorL);                                   // link the motor to the sensor
  
  sensorR.init();                                                // initialise magnetic sensor hardware
  motorR.linkSensor(&sensorR);                                   // link the motor to the sensor

  Serial.println("Calibrating gyro...");
  angle_setup();

  motorL.voltage_power_supply = 12;
  motorL.controller = ControlType::voltage;                      // set control loop type to be used
  motorL.voltage_sensor_align = 4;                               // aligning voltage 
  motorL.foc_modulation = FOCModulationType::SpaceVectorPWM;     // choose FOC modulation (optional)
  motorL.init();                                                 // initialize motor
  motorL.initFOC();                                              // align encoder and start FOC
  
  motorR.voltage_power_supply = 12;
  motorR.controller = ControlType::voltage;                      // set control loop type to be used
  motorR.voltage_sensor_align = 4;                               // aligning voltage 
  motorR.foc_modulation = FOCModulationType::SpaceVectorPWM;     // choose FOC modulation (optional)
  motorR.init();                                                 // initialize motor
  motorR.initFOC();                                              // align encoder and start FOC
}

void loop() {
  // ~1ms 
  motorL.loopFOC();
  motorR.loopFOC();

  if (test) {
    motorL.move(5 * (motorR.shaft_angle - motorL.shaft_angle));
    motorR.move(5 * (motorL.shaft_angle - motorR.shaft_angle));
  } else {
    //Tuning();
    bluetooth();
    angle_calc();
    gyroY = GyY / 131.0; // Convert to deg/s
    
    if (loop_count++ > loop_time) { 
      float target_voltage = 0;
      if (vertical) {      
        target_voltage = controlleR(robot_angle, gyroY, robot_speed, robot_position);
        robot_speed = (motorL.shaftVelocity() + motorR.shaftVelocity()) / 2;
        robot_position += (motorL.shaftVelocity() + motorR.shaftVelocity()) / 20 - move_Speed;
        robot_position = constrain(robot_position, -30, 30);
        Serial.println(robot_position);
        motorL.move(target_voltage + rot_Speed);
        motorR.move(target_voltage - rot_Speed);
      } else {
        motorL.move(target_voltage);
        motorR.move(target_voltage);
        robot_position = 0;
      }
      loop_count = 0;
    }
  }
  if (loop2_count++ > 8000) {
    test = false;
    loop2_count = 0;
  }
}

int sign(int x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  if (x = 0) return 0;
}

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

float controlleR(float p_angle, float p_vel, float s_vel, float a_vel) {
  float u =  K1 * p_angle + K2 * p_vel + K3 * s_vel + K4 * a_vel;
  if (abs(u) > 12 * 0.8) 
    u = sign(u) * 12 * 0.8;
  return u;
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay(100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(3);
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = ");  Serial.println(GyY_offset);
}

void angle_calc() {

   // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                     // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  AcX = Wire.read() << 8 | Wire.read(); 
  //Serial.println(AcX);
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3F);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  AcZ = Wire.read() << 8 | Wire.read(); 
  //Serial.println(AcZ);
  Wire.beginTransmission(MPU6050);
  Wire.write(0x45);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  GyY = Wire.read() << 8 | Wire.read(); 

  AcX += AcX_offset;
  AcZ += AcZ_offset;
  GyY += GyY_offset;

  robot_angle += GyY * 6.07968E-5;                      
  Acc_angle =  atan2(-AcX, AcZ) * 57.2958;    
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);

  if (abs(robot_angle) > 40) vertical = false;
  if (abs(robot_angle) < 1) vertical = true;

  //Serial.println(robot_angle);
}

int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    K1 += 0.02;
      if (cmd == '-')    K1 -= 0.02;
      printValues();
      break;
    case 'd':
      if (cmd == '+')    K2 += 0.002;
      if (cmd == '-')    K2 -= 0.002;
      printValues();
      break;      
    case 's':
      if (cmd == '+')    K3 += 0.01;
      if (cmd == '-')    K3 -= 0.01;
      printValues();
      break;
    case 'a':
      if (cmd == '+')    K4 += 0.01;
      if (cmd == '-')    K4 -= 0.01;
      printValues();
      break;  
  }
}

void printValues() {
  Serial.print("K1: "); Serial.print(K1);
  Serial.print(" K2: "); Serial.print(K2,3);
  Serial.print(" K3: "); Serial.print(K4);
  Serial.print(" K4: "); Serial.println(K3);
}

