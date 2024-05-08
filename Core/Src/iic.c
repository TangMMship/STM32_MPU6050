//
// Created by TangM on 2024/5/8.
//
#include "iic.h"
#include "stm32f1xx.h"
#include "tim.h"
void Delay_uS(uint16_t uS_Count)
{
    uint16_t counter = 0;                         /*暂存定时器的计数值*/
    __HAL_TIM_SET_AUTORELOAD(&htim1, uS_Count);   /*设置定时器自动加载值，到该值后重新计数*/
    __HAL_TIM_SET_COUNTER(&htim1, 0);             /*设置定时器初始值*/
    HAL_TIM_Base_Start(&htim1);                   /*启动定时器*/
    while(counter < uS_Count)                     /*直到定时器计数从0计数到uS_Count结束循环,刚好uS_Count uS*/
    {
        counter = __HAL_TIM_GET_COUNTER(&htim1);    /*获取定时器当前计数*/
    }
    HAL_TIM_Base_Stop(&htim1);                    /*停止定时器*/
}
void IIC_Start()
{
    IIC_SDA=1;
    Delay_uS(10);
    IIC_SCL=1;
    //修改了Delay_uS(数  此时延时单位us
    Delay_uS(10);
    IIC_SDA=0;
    Delay_uS(10);   //开始信号
    IIC_SCL=0;            //SCL拉低可以传输信号
    Delay_uS(10);
}
/*IIC停止信号
 * 当SCL为高时 SDA跳变为高
 *因为正常传输信号是SCL为高时SDA都是保持,所以开始结束SDA在此跳变可以区分
 *要保证SCL为高是SDA先是低再是高，我们不能再SCL为高时改变，不然我们直接先把SDA拉低可能就是开始信号了
 *所以先把SCL，SDA拉低，再把SCL拉高，这样SCL高的时候SDA初始状态就是低，此时再拉高
 */
void IIC_Stop()
{

    IIC_SDA=0;
    Delay_uS(10);
    IIC_SCL=1;
    Delay_uS(10);
    IIC_SDA=1;
    Delay_uS(10);
}

/*等待应答
 *返回1无应答
 *返回0应答
 */
uint8_t Wait_ack()
{
    uint8_t count=0;   //滤波系数  防止电平跳变的影响
    IIC_SDA=1;       //不用SDA时都要为高阻态，只有传输信号的设备SDA可以拉低，因为是开漏，所以SDA整条线都会拉低,因此一次只能有一个设备发送消息
    Delay_uS(4);
    IIC_SCL=1;       //读数据
    Delay_uS(6);
    while(IIC_readSDA)
    {
        count++;
        if(count>250)       //一直没应答
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL=0;  //继续接收
    return 0;
}

/*发送数据
 *
 */
void IIC_Sendbyte(uint8_t txb)
{
    uint8_t i;
    //IIC_SCL=0;
    for(i=0;i<8;i++)
    {
        IIC_SDA=(txb&0x80)>>7;      //iic从最高位开始传输，取最高位再移动到第一位给SDA赋值
        txb<<=1;
        Delay_uS(10);
        IIC_SCL=1;
        Delay_uS(10);
        IIC_SCL=0;
        Delay_uS(10);
    }
}

/*
 * 接受数据
 * ack为1发送应答
 * ack为0不应答
 */
uint8_t IIC_Readbyte(unsigned char ack)
{

    uint8_t rebyte,i;
    rebyte=0;
    IIC_SDA=1;          //不干扰
    Delay_uS(10);
    for (i = 0; i < 8; i ++)
    {
        IIC_SCL=1;
        Delay_uS(10);
        if (IIC_readSDA == 1){rebyte |= (0x80 >> i);}
        IIC_SCL=0;
        Delay_uS(10);
    }

    return rebyte;
}

/*
 *
 */
void IIC_Sendack()
{
    IIC_SCL=0;
    IIC_SDA=0;                  //拉高SDA产生非应答信号
    Delay_uS(4);
    IIC_SCL=1;
    Delay_uS(4);         //完成应答
    IIC_SCL=0;                  //等待下次信号
}
void IIC_Nsendack()
{

    IIC_SDA=1;                  //拉高SDA产生非应答信号
    Delay_uS(10);
    IIC_SCL=1;
    Delay_uS(10);         //完成应答
    IIC_SCL=0;                  //等待下次信号
}