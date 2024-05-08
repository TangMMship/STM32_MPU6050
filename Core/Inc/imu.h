//
// Created by TangM on 2024/5/8.
//

#ifndef STM32_MPU6050_IMU_H
#define STM32_MPU6050_IMU_H
#include "math.h"

typedef struct{
    float AX;
    float AY;
    float AZ;
    float GX;
    float GY;
    float GZ;
}param_imu;

typedef struct{
    float Pitch;
    float Roll;
    float Yaw;
}param_Angle;

extern param_imu imu_data;
extern param_Angle imu_Angle;
void IMU_getEuleranAngles(void);

#endif //STM32_MPU6050_IMU_H
