#include "config.h"

// 卡尔曼内部变量
float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.03;
float kalmanAngle = 0, bias = 0;
float P[2][2] = {{1, 0}, {0, 1}};

float kalmanUpdate(float newAngle, float newRate, float dt) {
  float rate = newRate - bias;
  kalmanAngle += dt * rate;
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyro * dt;
  float S = P[0][0] + R_angle;
  float K[2] = {P[0][0] / S, P[1][0] / S};
  float y = newAngle - kalmanAngle;
  kalmanAngle += K[0] * y;
  bias += K[1] * y;
  float P00_temp = P[0][0], P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  return kalmanAngle;
}