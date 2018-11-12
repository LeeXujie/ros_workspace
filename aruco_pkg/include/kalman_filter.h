#ifndef __KALMAN_H__
#define __KALMAN_H__
#include <iostream>

// 一维滤波器信息结构体
typedef struct {
  float filterValue; // k-1时刻的滤波值，即是k-1时刻的值
  float kalmanGain;  // Kalamn增益
  float A;           // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
  float H;           // z(n)=H*x(n)+w(n),w(n)~N(0,R)
  float Q;           //预测过程噪声偏差的方差
  float R; 			 //测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
  float P; 			 //估计误差协方差
} KalmanInfo;

class Kalman {
public:
  /**
  * @brief Init_KalmanInfo   初始化滤波器的初始值
  * @param info  滤波器指针
  * @param Q 预测噪声方差 由系统外部测定给定
  * @param R 测量噪声方差 由系统外部测定给定
  */
  void Init_KalmanInfo(KalmanInfo *info, float Q, float R) {
    info->A = 1; //标量卡尔曼
    info->H = 1; //
    info->P = 1; //后验状态估计值误差的方差的初始值（不要为0问题不大）
    info->Q = Q; //预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
    info->R = R; //测量（观测）噪声方差 可以通过实验手段获得
    info->filterValue = 0; // 测量的初始值
  }

  float m_KalmanFilter(KalmanInfo *kalmanInfo, float lastMeasurement) {

    //预测：做出先验估计 x[n|n-1] = A*x[n-1|n-1]
    float predictValue = kalmanInfo->A * kalmanInfo->filterValue; 

    //向前推算协方差 P[n|n-1] = A^2*P[n-1|n-1]+Q，对于一维情况通常A=1,P[n|n-1]=P[n-1|n-1]+Q
    kalmanInfo->P = kalmanInfo->A * kalmanInfo->A * kalmanInfo->P + kalmanInfo->Q; 

    //计算卡尔曼增益 K[n] = P[n|n-1] * H' / {H P[n|n-1] H’ + R}
    kalmanInfo->kalmanGain = kalmanInfo->P * kalmanInfo->H / (kalmanInfo->P * kalmanInfo->H * kalmanInfo->H + kalmanInfo->R);
    
	//做出后验估计，计算滤波值 x[n|n] = x[n|n-1]+K[n]*{z[n]-x[n|n-1]}
    kalmanInfo->filterValue = predictValue + (lastMeasurement - predictValue) * kalmanInfo->kalmanGain; 

    //更新后验估计 P[n|n]={1-K[n]*H}*P[n|n-1]
    kalmanInfo->P = (1 - kalmanInfo->kalmanGain * kalmanInfo->H) * kalmanInfo->P;

    return kalmanInfo->filterValue;
  }
};

#endif