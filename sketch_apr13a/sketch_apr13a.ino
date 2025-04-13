#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define FREQUENCY_HZ   50
#define INTERVAL_MS    (1000 / (FREQUENCY_HZ + 1))
#define ACC_RANGE      1  // 0: +/-2G, 1: +/-4G

// Convert factor g to m/s^2 --> [-32768, +32767] --> [-2g, +2g]
#define CONVERT_G_TO_MS2  (9.81 / (16384.0 / (1. + ACC_RANGE)))

static unsigned long last_interval_ms = 0;

MPU6050 imu;
int16_t ax, ay, az;

void setup(){
    Serial.begin(115200);
    Serial.println("Initializing I2C devices...");
    Wire.begin();
    imu.initialize();
    delay(10);

    // verify connection
    // if (imu.testConnection()) {
    //     Serial.println("IMU connected");
    // } else {
    //     Serial.println("IMU Error");
    // }

    delay(300);

    // Set MCU 6050 OffSet Calibration
    imu.setXAccelOffset(-4732);  // : -412 592 1375 110 -35 -16
    imu.setYAccelOffset(4703);
    imu.setZAccelOffset(8867);
    imu.setXGyroOffset(61);
    imu.setYGyroOffset(-73);
    imu.setZGyroOffset(35);

    imu.setFullScaleAccelRange(ACC_RANGE);

}

void loop() {

  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();

    // read raw accel/gyro measurements from device
    imu.getAcceleration(&ax, &ay, &az);

    // converting to m/s²
    float ax_m_s2 = ax * CONVERT_G_TO_MS2;
    float ay_m_s2 = ay * CONVERT_G_TO_MS2;
    float az_m_s2 = az * CONVERT_G_TO_MS2;

    Serial.print(ax_m_s2);
    Serial.print("\t");
    Serial.print(ay_m_s2);
    Serial.print("\t");
    Serial.println(az_m_s2);
  }

}

