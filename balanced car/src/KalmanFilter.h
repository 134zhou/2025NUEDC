#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

struct KalmanFilter {
  float angle;       // 当前角度估计
  float bias;        // 当前偏差估计（未使用时可省）
  float P;           // 协方差
  float Q_angle;     // 过程噪声协方差
  float R_measure;   // 测量噪声协方差
};

void initKalman(KalmanFilter &kf, float q, float r);
float updateKalman(KalmanFilter &kf, float newAngle, float newRate, float dt); 
#endif