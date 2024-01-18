#include <Adafruit_MMC56x3.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define FREQ_HZ 100
#define BUS_COUNT 5

bool started;
int period_ms;
unsigned long next_sample_time;

Adafruit_MPU6050 mpu;
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
bool mpu_ready[5] = {0, 0, 0, 0, 0};
bool mmc_ready[5] = {0, 0, 0, 0, 0};

sensors_event_t imu_acc[5], imu_gyro[5], imu_temp[5], mag_event[5];
char buffer[512];


void switchBus(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(0x1 << bus);
  byte error = Wire.endTransmission();
}

// Set up the communication, I2C devices, etc.
void setup(void) {
  Wire.begin();

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  started = false;
  period_ms = 1000 / FREQ_HZ;
  next_sample_time = 0;

  for (uint8_t bus = 0; bus < BUS_COUNT; bus++) {
    switchBus(bus);

    // Try to initialize!
    for (int i = 0; i < 10; i++) {
      if (mpu.begin(0x69)) { // Nice.
        mpu_ready[bus] = 1;
        Serial.print("Connected IMU ");
        Serial.println(bus);

        // MPU6050_RANGE_2_G, MPU6050_RANGE_4_G, MPU6050_RANGE_8_G, MPU6050_RANGE_16_G
        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        // MPU6050_RANGE_250_DEG, MPU6050_RANGE_500_DEG, MPU6050_RANGE_1000_DEG, MPU6050_RANGE_2000_DEG
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        // 10, 21, 44, 94, 184, or 260
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);

        break;
      }
      Serial.print(bus);
      Serial.println(" IMU not initializing!!!");
      delay(10);
    }

    for (int i = 0; i < 10; i++) {
      if (mmc.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
        mmc_ready[bus] = 1;
        Serial.print("Connected Magnetometer ");
        Serial.println(bus);

        mmc.setDataRate(255);
        mmc.setContinuousMode(1);

        break;
      }
      Serial.print(bus);
      Serial.println(" Magnetometer not initializing!!!");
      delay(10);
    }

    delay(10);
  }

}

void loop() {

  if (!started) {
    started = true;
    next_sample_time = millis();
  }

  if (!started)
    return;

  unsigned long loop_start_time = millis();
  next_sample_time = loop_start_time + period_ms;

  float timestamp = loop_start_time / 1000.0f;

  for(uint8_t bus = 0; bus < BUS_COUNT; bus++) {
    switchBus(bus);

    // Get new sensor events with the readings.
    if (mpu_ready[bus]) {
      mpu.getEvent(&imu_acc[bus], &imu_gyro[bus], &imu_temp[bus]);
    }
    if (mmc_ready[bus]) {
      mmc.getEvent(&mag_event[bus]);
    }
  }

  char *dp = buffer;
  dp += sprintf(dp, "%.4f,", timestamp);
  for(uint8_t bus = 0; bus < BUS_COUNT; bus++) {
      dp += sprintf(dp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f ",
      imu_acc[bus].acceleration.x, imu_acc[bus].acceleration.y, imu_acc[bus].acceleration.z,
      imu_gyro[bus].gyro.x, imu_gyro[bus].gyro.y, imu_gyro[bus].gyro.z,
      mag_event[bus].magnetic.x, mag_event[bus].magnetic.y, mag_event[bus].magnetic.z);
  }
  Serial.println(buffer);

  unsigned long loop_end_time = millis();

  if (loop_end_time < next_sample_time) {
    delay(next_sample_time - loop_end_time);
  }

  Serial.flush();
}
