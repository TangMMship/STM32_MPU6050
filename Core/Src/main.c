/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "retarget.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define IIC_SCL    PBout(10) //SCL输出配置  对相应的电器属性的寄存器写值
#define IIC_SDA    PBout(11) //SDA输出配置
#define LED   PCout(13) //SDA输出配置
#define MPU6050_ADDRESS		0xD0
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void IIC_Start();
void IIC_Stop();
void IIC_Sendbyte(uint8_t txb);
uint8_t Wait_ack();
uint8_t IIC_Readbyte(unsigned char ack);
void IIC_Sendack();
void IIC_Nsendack();
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
int addr_search();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

//IO口输出寄存器地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C
//IO口输入寄存器地址映射
#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08

//IO口操作,只对单一的IO口!
//确保n的值为0~15
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//阻塞方式打印,串口1
    return ch;
}
/*********************************************************************************************************/
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

uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    RetargetInit(&huart1);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    MPU6050_Init();
    ID = MPU6050_GetID();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      //printf("addrs:%d\r\n",addr_search());
      LED=1;
      for(int i=0;i<100000;i++)Delay_uS(10);
      LED=0;
      for(int i=0;i<100000;i++)Delay_uS(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void IIC_Start()
{
    IIC_SDA=1;
    IIC_SCL=1;
    //修改了Delay_uS(数  此时延时单位us
    Delay_uS(10);
    IIC_SDA=0;
    Delay_uS(10);   //开始信号
    IIC_SCL=0;            //SCL拉低可以传输信号
}
/*IIC停止信号
 * 当SCL为高时 SDA跳变为高
 *因为正常传输信号是SCL为高时SDA都是保持,所以开始结束SDA在此跳变可以区分
 *要保证SCL为高是SDA先是低再是高，我们不能再SCL为高时改变，不然我们直接先把SDA拉低可能就是开始信号了
 *所以先把SCL，SDA拉低，再把SCL拉高，这样SCL高的时候SDA初始状态就是低，此时再拉高
 */
void IIC_Stop()
{
    IIC_SCL=0;
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
    while(IIC_SDA)
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
    IIC_SCL=0;
    for(i=0;i<8;i++)
    {
        IIC_SDA=(txb&0x80)>>7;      //iic从最高位开始传输，取最高位再移动到第一位给SDA赋值
        txb<<=1;
        IIC_SCL=1;
        Delay_uS(4);
        IIC_SCL=0;
        Delay_uS(3);
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
    for(i=0;i<8;i++)
    {
        IIC_SCL=0;
        Delay_uS(6);    //一般iic传输速率100kb，差不多1.25us  高速400kb
        IIC_SCL=1;
        rebyte<<=1;
        rebyte|=IIC_SDA;
        Delay_uS(6);
    }
    if(ack)
    {
        IIC_Sendack();
    }
    else
    {
        IIC_Nsendack();
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
    IIC_SCL=0;
    IIC_SDA=1;                  //拉高SDA产生非应答信号
    Delay_uS(4);
    IIC_SCL=1;
    Delay_uS(4);         //完成应答
    IIC_SCL=0;                  //等待下次信号
}
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    IIC_Start();
    IIC_Sendbyte(MPU6050_ADDRESS);
    if(Wait_ack()==1){printf("err_ack\r\n");}
    else
//        printf("%d\r\n",)
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
    IIC_Stop();

    return Data;
}
void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
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
int addr_search()
{
    uint8_t addr=0,i=0,addrs;
    IIC_Start();
    for(i=0;i<255;i++)
    {
        addrs=addr<<1;
        IIC_Sendbyte(MPU6050_ADDRESS);
        if(Wait_ack()==0)
        {
            printf("addrs:%d\r\n",addrs);
            return 1;

        }
        addr++;
    }
    IIC_Stop();
    return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
