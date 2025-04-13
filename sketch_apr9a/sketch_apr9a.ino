
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Configuration
int buffersize = 1000;
int acel_deadzone = 8;
int gyro_deadzone = 1;

MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// Setup
void setup() {
  // Start I2C on ESP32 (GPIO21 = SDA, GPIO22 = SCL)
  Wire.begin(21, 22);  // Change these if you're using different pins
  Serial.begin(115200);
  
  accelgyro.initialize();

  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()) {
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
  }
  while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println("\nMPU6050 Calibration Sketch");
  delay(2000);
  Serial.println("Your MPU6050 should be placed in horizontal position.");
  Serial.println("Don't touch it until you see a finish message.\n");
  delay(3000);

  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);

  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
}

// Functions
void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) {
      buff_ax += ax;
      buff_ay += ay;
      buff_az += az;
      buff_gx += gx;
      buff_gy += gy;
      buff_gz += gz;
    }

    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }

    i++;
    delay(2);
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  while (true) {
    int ready = 0;

    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset -= mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset -= mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset += (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= gyro_deadzone) ready++;
    else gx_offset -= mean_gx / (gyro_deadzone + 1);

    if (abs(mean_gy) <= gyro_deadzone) ready++;
    else gy_offset -= mean_gy / (gyro_deadzone + 1);

    if (abs(mean_gz) <= gyro_deadzone) ready++;
    else gz_offset -= mean_gz / (gyro_deadzone + 1);

    if (ready == 6) break;
  }
}

void loop() {
  meansensors();
  calibration();

  Serial.println("\nCalibration Complete!\n");

  Serial.print("Accelerometer Offsets: ");
  Serial.print(ax_offset); Serial.print(", ");
  Serial.print(ay_offset); Serial.print(", ");
  Serial.println(az_offset);

  Serial.print("Gyroscope Offsets: ");
  Serial.print(gx_offset); Serial.print(", ");
  Serial.print(gy_offset); Serial.print(", ");
  Serial.println(gz_offset);

  while (1);
    // Freeze loop after calibration
}
