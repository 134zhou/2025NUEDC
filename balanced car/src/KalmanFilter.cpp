#include<KalmanFilter.h>

void initKalman(KalmanFilter &kf, float q, float r) 
{
  kf.angle = 0;
  kf.bias = 0;
  kf.P = 1;
  kf.Q_angle = q;
  kf.R_measure = r;
}

float updateKalman(KalmanFilter &kf, float newAngle, float newRate, float dt) 
{
  // 预测
  kf.angle += dt * (newRate - kf.bias);
  kf.P += kf.Q_angle;
  
  // 计算卡尔曼增益
  float K = kf.P / (kf.P + kf.R_measure);

  // 更新
  kf.angle += K * (newAngle - kf.angle);
  kf.P *= (1 - K);

  return kf.angle;
}
