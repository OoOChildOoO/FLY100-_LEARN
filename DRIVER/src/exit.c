
#include "stm32f4xx.h"
#include "exit.h"
#include "led.h"
#include "delay.h"
#include "stdio.h"
#define test_exit 0
/****************************************************************************************************
*函  数: void EXTI_GPIOConfig(void)
*功  能: 配置与NRF的IRQ相连的IO
*参  数: 无
*返回值：无
*备  注: 无
****************************************************************************************************/
void EXTI_GPIOConfig(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
	#if test_exit
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // GPIOB 时钟使能 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //GPIO输出速度
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//仅仅对配置成输入时有效
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	#else
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // GPIOA 时钟使能 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //GPIO输出速度
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//仅仅对配置成输入时有效
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	#endif
	
}

/****************************************************************************************************
*函  数: void Exti_Init(void)
*功  能: 外部中断
*参  数: 无
*返回值：无
*备  注: NRF的IRQ引脚触发的外部中断
****************************************************************************************************/
void Exti_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructer;
	#if test_exit 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTIT,ENABLE); 		//使能外部中断时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //使能系统配置时钟   

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2); //将中断线EXTI Line2与PB2连接
	EXTI_GPIOConfig();
	EXTI_DeInit(); //复位外部中断
	 
	//初始化中断线2
	EXTI_InitStructer.EXTI_Line = EXTI_Line2; //选择中断线2
	EXTI_InitStructer.EXTI_Mode = EXTI_Mode_Interrupt; //中断模式
	EXTI_InitStructer.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
	EXTI_InitStructer.EXTI_LineCmd = ENABLE; //中断线2使能
	EXTI_Init(&EXTI_InitStructer);
	#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTIT,ENABLE); 		//使能外部中断时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //使能系统配置时钟   

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1); //将中断线EXTI Line1与PB2连接
	EXTI_GPIOConfig();
	EXTI_DeInit(); //复位外部中断
	 
	//初始化中断线2
	EXTI_InitStructer.EXTI_Line = EXTI_Line1; //选择中断线1
	EXTI_InitStructer.EXTI_Mode = EXTI_Mode_Interrupt; //中断模式
	EXTI_InitStructer.EXTI_Trigger = EXTI_Trigger_Falling; //傻上升降沿触发
	EXTI_InitStructer.EXTI_LineCmd = ENABLE; //中断线2使能
	EXTI_Init(&EXTI_InitStructer);
	#endif
}

