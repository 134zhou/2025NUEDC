#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <KalmanFilter.h>
#include <PID.h>

Adafruit_MPU6050 mpu;
float ax, ay, az, gx, gy, gz;
KalmanFilter kfPitch, kfRoll;
float yaw = 0;
bool flag = false;
uint8_t n = 0;
float kalPitch;
float kalRoll;

hw_timer_t *timer = NULL;

void Interrupt_1ms(void)
{
  flag = true;
}


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) 
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  initKalman(kfPitch, 0.0025, 0.03);
  initKalman(kfRoll, 0.0025, 0.03);
  //定时器中断
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &Interrupt_1ms, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
}

void loop() 
{
  if (flag)
  {
    n++;
    flag = false;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dt = 0.01;

    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;

    // 加速度角度估算
    float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float accRoll  = atan2(ay, az) * 180.0 / PI;


    float gyroX = gx* 180.0 / PI;
    float gyroY = gy* 180.0 / PI;
    float gyroZ = gz* 180.0 / PI;

    kalPitch = updateKalman(kfPitch, accPitch, gyroY, dt);
    kalRoll  = updateKalman(kfRoll, accRoll,  gyroX, dt);

    yaw += gyroZ * dt;   
  }
  if (n==100)
  {
    Serial.print("Pitch: ");
    Serial.print(kalPitch, 2);
    Serial.print(" | Roll: ");
    Serial.print(kalRoll, 2);
    Serial.print(" | Yaw: ");
    Serial.println(yaw, 2);
    n = 0;
  }
  

}
