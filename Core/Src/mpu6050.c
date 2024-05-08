//
// Created by TangM on 2024/5/8.
//
#include "mpu6050.h"
#include "iic.h"
#include "mpu6050_reg.h"
#include "stdio.h"
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    IIC_Start();
    IIC_Sendbyte(MPU6050_ADDRESS);
    int ack=Wait_ack();
    if(ack==1){printf("err_ack\r\n");}
    else
        printf("%d\r\n",ack);
    IIC_Sendbyte(RegAddress);
    if(Wait_ack())printf("err_ack\r\n");
    IIC_Sendbyte(Data);
    if(Wait_ack())printf("err_ack\r\n");
    IIC_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    IIC_Start();
    IIC_Sendbyte(MPU6050_ADDRESS);
    if(Wait_ack()){printf("err_ack\r\n");}
    IIC_Sendbyte(RegAddress);
    if(Wait_ack()){printf("err_ack\r\n");}

    IIC_Start();
    IIC_Sendbyte(MPU6050_ADDRESS | 0x01);
    if(Wait_ack()==1){printf("err_ack\r\n");}
    Data = IIC_Readbyte(0);
    IIC_Nsendack();
    IIC_Stop();

    return Data;
}
void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0X80);	//复位MPU6050
    //等待复位完成
    HAL_Delay(100);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}



//一开始只有一个起始在循环外面，发的遍历寻址没有停止信号，相当于给第一个地址一直发消息，因为没停止信号其他信号不知道是寻址。
int addr_search()
{
    uint8_t addr=0,i=0,addrs,ack;

    for(i=0;i<255;i++)
    {
        IIC_Start();
        addrs=addr<<1;
        IIC_Sendbyte(addrs);
        ack=Wait_ack();
        if(ack==0)
        {
            IIC_Stop();
            return addrs;

        }
        else
            IIC_Stop();
        addr++;
    }

    return 0;
}