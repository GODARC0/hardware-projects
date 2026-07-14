#include <MPU6500_WE.h>
#include <Wire.h>

#define MPU6500_ADDR 0x68  // I2C address (0x69 if ADO is High)

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

void setup() {
  Serial.begin(115200);
  Wire.begin(25, 26); // Explicitly define ESP32 I2C pins (SDA=21, SCL=22)

  Serial.println("Initializing MPU6500...");

  if (!myMPU6500.init()) {
    Serial.println("MPU6500 connection failed! Check your wiring.");
    while (1);
  }

  Serial.println("MPU6500 connected successfully!");

  /* Calibrate the sensor on startup (Keep it completely still on a flat surface!) */
  myMPU6500.autoOffsets(); 
  
  // Set Accelerometer Range (Options: MPU6500_ACC_RANGE_2G, 4G, 8G, 16G)
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  
  // Set Gyro Range (Options: MPU6500_GYRO_RANGE_250, 500, 1000, 2000)
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  
  // FIXED: Gyroscope filter function takes NO arguments
  myMPU6500.enableGyrDLPF();            
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6); 
  
  // FIXED: Accelerometer filter function REQUIRES 'true'
  myMPU6500.enableAccDLPF(true);        
  myMPU6500.setAccDLPF(MPU6500_DLPF_6); 
}

void loop() {
  xyzFloat gForce = myMPU6500.getGValues();
  xyzFloat gyroValues = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();

  // Print Accelerometer Data (in g)
  Serial.print("Accel X: "); Serial.print(gForce.x);
  Serial.print(" | Y: "); Serial.print(gForce.y);
  Serial.print(" | Z: "); Serial.print(gForce.z);
  Serial.println(" g");

  // Print Gyroscope Data (in degrees per second)
  Serial.print("Gyro  X: "); Serial.print(gyroValues.x);
  Serial.print(" | Y: "); Serial.print(gyroValues.y);
  Serial.print(" | Z: "); Serial.print(gyroValues.z);
  Serial.println(" °/s");

  // Print Temperature
  Serial.print("Temperature: "); Serial.print(temp);
  Serial.println(" °C");

  Serial.println("------------------------------------");
  delay(500); 
}