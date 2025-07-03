#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <KalmanFilter.h>
#include <PID.h>

// SDA = 21;
// SCL = 22;

#define wheel_1_P 12
#define wheel_1_N 13
#define wheel_2_P 2
#define wheel_2_N 4

Adafruit_MPU6050 mpu;
float ax, ay, az, gx, gy, gz;
KalmanFilter kfPitch, kfRoll;
float yaw = 0;
bool flag = false;
uint8_t n = 0;
float kalPitch;
float kalRoll;

hw_timer_t *timer = NULL;
PID_Param pid;

void Interrupt_1ms(void)
{
  flag = true;
}

void Init_motor_control();
void motor(int wheel_1_value,int wheel_2_value);
void angle_update();

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) 
  {
    Serial.println("Failed to find MPU6050 chip");
    while(1)
    {
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

  Init_motor_control();
  PID_Init(&pid, 0.5, 0.005, 0, 255, -255);
}

void loop() 
{
  if (flag)
  {
    angle_update();
    PID_Calculate(&pid,kalPitch);
    Serial.println(pid.output);
    motor(pid.output,pid.output);
  }

}

void Init_motor_control()
{
  pinMode(wheel_1_P,OUTPUT);
  pinMode(wheel_1_N,OUTPUT);
  ledcSetup(0,20000,9);
  ledcAttachPin(wheel_1_P,0);
  ledcWrite(0,0);

  ledcSetup(2,20000,9);
  ledcAttachPin(wheel_1_N,2);
  ledcWrite(2,0);

  pinMode(wheel_2_P,OUTPUT);
  pinMode(wheel_2_N,OUTPUT);
  ledcSetup(4,20000,9);
  ledcAttachPin(wheel_2_P,4);
  ledcWrite(4,0);

  ledcSetup(6,20000,9);
  ledcAttachPin(wheel_2_N,6);
  ledcWrite(6,0);
}

//输入值-256-256，正的正转，负的反转
void motor(int wheel_1_value,int wheel_2_value)
{
  if (wheel_1_value >= 0)
  {
    ledcWrite(0,wheel_1_value+256);
    ledcWrite(2,0);
  }
  else
  {
    ledcWrite(0,0);
    ledcWrite(2,256-wheel_1_value);
  }

  if (wheel_2_value >= 0)
  {
    ledcWrite(4,wheel_2_value+256);
    ledcWrite(6,0);
  }
  else
  {
    ledcWrite(4,0);
    ledcWrite(6,256-wheel_2_value);
  }
}

void angle_update()
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