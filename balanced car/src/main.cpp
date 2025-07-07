#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
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
esp_now_peer_info_t peerInfo;

// 发送结构体类型
typedef struct struct_message
{
  char command[32];
  int Integer;
  float float_value;
  bool bool_value;
} struct_message;

struct_message myData;

// 接收设备的 MAC 地址
uint8_t ControllerAddress[] = {0x78, 0x21, 0x84, 0x9A, 0x3C, 0x64};

void Interrupt_1ms(void)
{
  flag = true;
}

void Init_motor_control();
void motor(int wheel_1_value,int wheel_2_value);
void angle_update();
void ESP_NOW_init();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
  delay(10);
  Serial.println("Adafruit MPU6050 test!");
  
  ESP_NOW_init();

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
    flag = false;
    angle_update();
    PID_Calculate(&pid,kalPitch);
    //Serial.println(pid.output);
    motor(pid.output,pid.output);
    strcpy(myData.command, "report pid");
    myData.float_value = pid.output;

    // 发送数据
    esp_err_t result = esp_now_send(ControllerAddress, (uint8_t *) &myData, sizeof(myData));

    // 检查数据是否发送成功
    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }
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
    // analogWrite(wheel_1_P,wheel_1_value);
    // analogWrite(wheel_1_N,0);
  }
  else
  {
    ledcWrite(0,0);
    ledcWrite(2,256-wheel_1_value);
    // analogWrite(wheel_1_P,0);
    // analogWrite(wheel_1_N,-wheel_1_value);
  }

  if (wheel_2_value >= 0)
  {
    ledcWrite(4,wheel_2_value+256);
    ledcWrite(6,0);
    // analogWrite(wheel_2_P,wheel_2_value);
    // analogWrite(wheel_2_N,0);
  }
  else
  {
    ledcWrite(4,0);
    ledcWrite(6,256-wheel_2_value);
    // analogWrite(wheel_2_P,0);
    // analogWrite(wheel_2_N,-wheel_2_value);
  }
}

void angle_update()
{
  n++;
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
    // Serial.print("Pitch: ");
    // Serial.print(kalPitch, 2);
    // Serial.print(" | Roll: ");
    // Serial.print(kalRoll, 2);
    // Serial.print(" | Yaw: ");
    // Serial.println(yaw, 2);
    n = 0;
  }
}

void ESP_NOW_init()
{
  // 初始化 ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 设置发送数据回调函数
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, ControllerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // 检查设备是否配对成功
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  strcpy(myData.command, "Default");
  myData.Integer = 0;
  myData.float_value = 0.0;
  myData.bool_value = false;
}

// 数据发送回调函数
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}