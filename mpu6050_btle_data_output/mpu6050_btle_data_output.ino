#include <ArduinoBLE.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_MMC56x3.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define FREQ_HZ 100
#define BUS_COUNT 5

int period_ms;
unsigned long next_sample_time;

Adafruit_MPU6050 mpu;
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
bool mpu_ready[5] = {0, 0, 0, 0, 0};
bool mmc_ready[5] = {0, 0, 0, 0, 0};


struct sensorDataPoints {
  float accx;
  float accy;
  float accz;
  float gyrx;
  float gyry;
  float gyrz;
  float magx;
  float magy;
  float magz;
};

struct dataPacket {
  unsigned long timestamp;
  struct sensorDataPoints allSensorData[BUS_COUNT];
};

int DATASIZE = sizeof(struct dataPacket);


BLEService touchTyperService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// BLE Characteristics - custom 128-bit UUIDs, read and writable by central
BLECharacteristic dataRawBytes("19B10004-E8F2-537E-4F6C-D104768A1214", BLENotify, DATASIZE);
BLEIntCharacteristic numImus("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead);


void switchBus(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(0x1 << bus);
  byte error = Wire.endTransmission();
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  
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

  // BTLE setup section.
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Set advertised local name and service UUID:
  BLE.setLocalName("Accelerometer Data - CAT");
  BLE.setAdvertisedService(touchTyperService);

  // Add the characteristics to the service.
  touchTyperService.addCharacteristic(dataRawBytes);
  touchTyperService.addCharacteristic(numImus);
  // add service
  BLE.addService(touchTyperService);

  // Set the initial values for the characteristics:
  struct dataPacket emptyPacket;
  dataRawBytes.writeValue(&emptyPacket, DATASIZE);
  numImus.writeValue(BUS_COUNT);

  // Start advertising.
  BLE.advertise();
  Serial.println("BLE Acceleration Peripheral");
}

void loop() {
  BLEDevice central = BLE.central();

  bool TEMP = true;

  // Listen for BLE peripherals to connect:
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address()); //prints the centrals MAC address:
    
    struct dataPacket packet;
 
    // if a central is connected to peripheral:
    while (central.connected()) {

      unsigned long loop_start_time = millis();
      next_sample_time = loop_start_time + period_ms;
      
      String data = String(loop_start_time / 1000.0f) + ",";

      packet.timestamp = loop_start_time;
      
      for(uint8_t bus = 0; bus < BUS_COUNT; bus++) {
        switchBus(bus);

        /* Get new sensor events with the readings */
        sensors_event_t imu_acc, imu_gyro, imu_temp, mag_event;
        if (mpu_ready[bus]) {
          mpu.getEvent(&imu_acc, &imu_gyro, &imu_temp);
          packet.allSensorData[bus].accx = imu_acc.acceleration.x;
          packet.allSensorData[bus].accy = imu_acc.acceleration.y;
          packet.allSensorData[bus].accz = imu_acc.acceleration.z;
          packet.allSensorData[bus].gyrx = imu_gyro.gyro.x;
          packet.allSensorData[bus].gyry = imu_gyro.gyro.y;
          packet.allSensorData[bus].gyrz = imu_gyro.gyro.z;

          data += String(imu_acc.acceleration.x) + "," + String(imu_acc.acceleration.y) + "," + String(imu_acc.acceleration.z) + "," +
                  String(imu_gyro.gyro.x) + "," + String(imu_gyro.gyro.y) + "," + String(imu_gyro.gyro.z) + ",";
        
        } else {
          // TODO should be zero init, make sure
        }

        if (mmc_ready[bus]) {
          mmc.getEvent(&mag_event);
          packet.allSensorData[bus].magx = mag_event.magnetic.x;
          packet.allSensorData[bus].magy = mag_event.magnetic.y;
          packet.allSensorData[bus].magz = mag_event.magnetic.z;
          data += String(mag_event.magnetic.x) + "," + String(mag_event.magnetic.y) + "," + String(mag_event.magnetic.z) + " ";
        } else {
          // TODO zero initialization should handle this case
        }
      }
      // Serial.print("Sending data: ");
      // Serial.println(data);
      // Serial.print("Took millis: ");
      // Serial.println(millis() - loop_start_time);

      if (!dataRawBytes.writeValue(&packet, DATASIZE))
        Serial.println("Writing data failed!");

      if (TEMP) {
        TEMP = false;
        int nLength = DATASIZE * 2;
        char pBuffer[nLength + 1];
        pBuffer[nLength] = 0;
        for (int i = 0; i < DATASIZE; i++)
        {
          sprintf(pBuffer + 2 * i, "%02X", ((char *)&packet)[i]);
        }
        Serial.println(String(pBuffer));
        Serial.println(dataRawBytes.valueLength());
      }

      unsigned long loop_end_time = millis();

      if (loop_end_time < next_sample_time) {
        delay(next_sample_time - loop_end_time);
      }

      Serial.flush();
    }
  }
}
