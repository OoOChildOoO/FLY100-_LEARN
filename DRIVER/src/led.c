
#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "stdlib.h"
#include "structconfig.h"

//#define LED1 GPIOB->BSRRL |=1<<9;	
//#define LED0 GPIOB->BSRRH |=1<<9; 
#define LED_RGB_ON GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define LED_RGB_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_9)

#define LED_USER_ON GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define LED_USER_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_8)

//跑马灯RGB三元色配出七彩跑马灯
static u32 Run_buf[][16] = 
{
	{0xFF0000,0,0,0,0xFF0000,0,0,0,0xFF0000,0,0,0,0xFF0000,0,0,0,},//红色
	{0xFFA500,0,0,0,0xFFA500,0,0,0,0xFFA500,0,0,0,0xFFA500,0,0,0,},//橙色
	{0x00FFFF,0,0,0,0x00FFFF,0,0,0,0x00FFFF,0,0,0,0x00FFFF,0,0,0,},//青色
	{0xFFFF00,0,0,0,0xFFFF00,0,0,0,0xFFFF00,0,0,0,0xFFFF00,0,0,0,},//黄色
	{0x0000FF,0,0,0,0x0000FF,0,0,0,0x0000FF,0,0,0,0x0000FF,0,0,0,},//蓝色
	{0x00FF00,0,0,0,0x00FF00,0,0,0,0x00FF00,0,0,0,0x00FF00,0,0,0,},//绿色
	{0xFF00FF,0,0,0,0xFF00FF,0,0,0,0xFF00FF,0,0,0,0xFF00FF,0,0,0,},//紫色
	{0xFFFFFF,0,0,0,0xFFFFFF,0,0,0,0xFFFFFF,0,0,0,0xFFFFFF,0,0,0,},//白色
	
};
uint8_t Run_flag=1;//跑马灯标志

	/*********************************************************************
*函  数：void LED_Init(void)
*功  能：初始化LED控制引脚
*参  数：无
*返回值: 无
*备  注: 无
*********************************************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //选择模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //输出类型  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_8);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //选择模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //输出类型  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOC,GPIO_Pin_15);
}

/***************************************************************************
*函  数：void LED_Run(void)
*功  能：指示MCU是否工作
*参  数：无
*返回值：无
*备  注: 无
***************************************************************************/

void LED_Run(void)
{
	static uint8_t flag = 1;
	if(flag)
	{
		flag = 0;
  	LED_USER_ON;
  }
	else
	{
		flag = 1;
	  LED_USER_OFF;
	}
}
/***************************************************************************
*函  数：void LED_Breath_User(void)
*功  能：呼吸灯 指示MCU是否工作
*参  数：无
*返回值：无
*备  注: LEDTimeCount * LEDTimeCount_Y * T(单位时间) = 周期   
				 每次亮灭的频率F 1/T*(单位时间) = 1000Hz
				 实际给人人眼的感受频率为  F/LEDTimeCount_Y = 50Hz
***************************************************************************/
void LED_Breath_User(void)
{
  static uint8_t LEDTimeCount=0;
	static uint8_t LEDTimeCount_Y=0;
 
	LEDTimeCount++;
	if(LEDTimeCount>20)
	{
		LEDTimeCount=0;
		LEDTimeCount_Y++;
		if(LEDTimeCount_Y>40)
			LEDTimeCount_Y=0;
	}
	/* 每隔1ms 进来一次 */
	if(LEDTimeCount_Y<20) /*上计数*/
	{
		if(LEDTimeCount<LEDTimeCount_Y)
			LED_USER_ON;
		else
			LED_USER_OFF;
   }
	else /*下减数*/
	{
		if(LEDTimeCount+20<LEDTimeCount_Y)
			LED_USER_OFF;
		else
			LED_USER_ON;
	}	
}


void RunTimer_Test(void)
{
	static uint8_t flag = 1;
	if(flag)
	{
		flag = 0;
  	GPIO_SetBits(GPIOC,GPIO_Pin_15);
  }
	else
	{
		flag = 1;
	  GPIO_ResetBits(GPIOC,GPIO_Pin_15);
	}
}

void RunTimer_Test2(void)
{
	static uint8_t flag = 1;
	if(flag)
	{
		flag = 0;
  	GPIO_SetBits(GPIOC,GPIO_Pin_14);
  }
	else
	{
		flag = 1;
	  GPIO_ResetBits(GPIOC,GPIO_Pin_14);
	}
}

/***************************************************************************
*函  数：void Write0(void)
*功  能：写0
*参  数：无
*返回值：无
*备  注: 不同的系统时钟频率下需要微调（默认HCLK=100MHz） __nop();一个nop为 10ns
***************************************************************************/
void Write0(void)
{
	uint8_t cnt1=4,cnt2 = 13;
	LED_RGB_ON;						//用setbit函数 大约60ns
	while(cnt1--)  //高电平300ns   150~450  
	{
		__nop();       //实测  //判断10ns  -1占40ns  一个循环60ns(含__nop();)
	}
  LED_RGB_OFF;						//用resetbit函数 大约60ns
	while(cnt2--)  //低电平900ns   750~1050
	{
		__nop();
	}
}

/***************************************************************************
*函  数：void Write1(void)
*功  能：写1
*参  数：无
*返回值：无
*备  注: 不同的系统时钟频率下需要微调（默认HCLK=100MHz）
***************************************************************************/
void Write1(void)
{
	uint8_t cnt1=9,cnt2 = 9;
	LED_RGB_ON;
	while(cnt1--) //高电平600ns			450~750
	{
		__nop();
	}
  LED_RGB_OFF;
	while(cnt2--) //低电平600ns			450~750
	{
		__nop();
	}
}

/***************************************************************************
*函  数：void Reset(void)
*功  能：RGB灯复位
*参  数：无
*返回值：无
*备  注: 无
***************************************************************************/
void Reset(void)
{
	//u16 cnt1 = 600;
	u16 cnt2 = 1;
	LED_RGB_OFF;
	//while(cnt1--)
	//__nop();
	delay_us(80);
  LED_RGB_ON;
	while(cnt2--)
	__nop();
}

/***************************************************************************
*函  数：Lvoid LED_Set_Byte(uint8_t data)
*功  能：写一个字节数据
*参  数：data 
*返回值: 无
*备  注: 无
***************************************************************************/
void LED_Set_Byte(uint8_t data)
{
	int i=0;
	for(i=0;i<8;i++)
	{
	  if((data<<i)&0x08)
		  Write1();
		else
			Write0();
	}
}

/***************************************************************************
*函  数：void RGB_LED_Colour_Set(uint8_t green,uint8_t red,uint8_t blue)
*功  能：设置一个灯的色彩
*参  数：green red blue，红绿蓝光所占比例大小,范围0~255
*返回值: 无
*备  注: 无
***************************************************************************/
void RGB_LED_Colour_Set(uint8_t green,uint8_t red,uint8_t blue)
{
	LED_Set_Byte(green);
	LED_Set_Byte(red);
	LED_Set_Byte(blue);
}

/**************************************************************************
*函  数：void RGB_LED_Runing(void)
*功  能：随机变换颜色
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_Rand(void)
{
	uint8_t i,red=0,green=0,blue=0;;
	for(i=0;i<4;i++)
	{
		green = rand()%18+2;//产生一个2~20的随机数
		red   = rand()%18+2;
		blue  = rand()%18+2;
    RGB_LED_Colour_Set(green,red,blue);//合成颜色
	}
 // Reset();        //复位显示 
//	delay_ms(300);	  
}

/**************************************************************************
*函  数：void RGB_LED_Runing(void)
*功  能：跑马灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_Runing(void)
{
	uint8_t i,red=0,green=0,blue=0;
	static uint8_t cnt = 0,wcnt = 0,times = 0;
	if(times++ >= 16)//跑完一圈换颜色
	{
	 times = 0; 
	 wcnt++;
	}
	for(i=0;i<4;i++)
	{
		if(cnt>4) cnt = 0;
		red   = ((Run_buf[wcnt][cnt]>>16)&0xff);
		green = ((Run_buf[wcnt][cnt]>>8)&0xff); 
		blue  = ((Run_buf[wcnt][cnt]>>0)&0xff);
		RGB_LED_Colour_Set(green,red,blue);//合成颜色
		cnt++;
	}
	if(wcnt==7) wcnt = 0;
 // Reset();        //复位显示 
 //	delay_ms(200);	  
}

/**************************************************************************
*函  数：void RGB_LED_Red(void)
*功  能：红灯
*参  数：无
*返回值：无
*备  注：还有很多种颜色可以自己慢慢调配
**************************************************************************/
void RGB_LED_Red(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0,0xFF,0);
	}
}

/**************************************************************************
*函  数：void RGB_LED_Orange(void)
*功  能：橙灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_Orange(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0xA5,0xFF,0x00);
	}
}

/**************************************************************************
*函  数：void RGB_LED_Yellow(void)
*功  能：黄灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_Yellow(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0xFF,0xFF,0);
	}
}

/**************************************************************************
*函  数：void RGB_LED_green(void)
*功  能：绿灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_green(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0xFF,0,0);
	}
}

/**************************************************************************
*函  数：void RGB_LED_Cyan(void)
*功  能：青灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_Cyan(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0xFF,0,0xFF);
	}
}

/**************************************************************************
*函  数：void RGB_LED_Blue(void)
*功  能：蓝灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_Blue(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0,0,0xFF);
	}
}

/**************************************************************************
*函  数：void RGB_LED_Violet(void)
*功  能：紫灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_Violet(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0x00,0xCD,0xCD);
	}
}

/**************************************************************************
*函  数：void RGB_LED_Violet(void)
*功  能：两红两绿
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_FLY(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
		if(i<2)
    RGB_LED_Colour_Set(0xFF,0,0);
		else
		RGB_LED_Colour_Set(0,0xFF,0);
	}
}

/**************************************************************************
*函  数：void RGB_LED_White(void)
*功  能：白灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_White(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0x0F,0x0F,0x0F);
	}
}

/**************************************************************************
*函  数：void RGB_LED_Off(void)
*功  能：关灯
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_LED_Off(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
    RGB_LED_Colour_Set(0,0,0);
	}
}

/**************************************************************************
*函  数：void GYRO_Offset_LED(void)
*功  能：陀螺仪校准完成蓝灯闪烁
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void GYRO_Offset_LED(void)
{
	 RGB_LED_Off();
	 RGB_LED_Blue();
	 delay_ms(100);
	 RGB_LED_Off();
	 delay_ms(100);
	 RGB_LED_Blue();
	 delay_ms(100);
	 RGB_LED_Off();
	 delay_ms(100);
	 RGB_LED_Blue();
	 delay_ms(100);
	 RGB_LED_Off();
}

/**************************************************************************
*函  数：void ACC_Offset_LED(void)
*功  能：加速度校准完成绿灯闪烁
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void ACC_Offset_LED(void)
{
	 RGB_LED_Off();
	 RGB_LED_green();
	 delay_ms(100);
	 RGB_LED_Off();
	 delay_ms(100);
	 RGB_LED_green();
	 delay_ms(100);
	 RGB_LED_Off();
	 delay_ms(100);
	 RGB_LED_green();
	 delay_ms(100);
	 RGB_LED_Off();
}

/**************************************************************************
*函  数：void BAR_Offset_LED(void)
*功  能：气压计校准完成紫灯闪烁
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void BAR_Offset_LED(void)
{
	 RGB_LED_Off();
	 RGB_LED_Violet();
	 delay_ms(100);
	 RGB_LED_Off();
	 delay_ms(100);
	 RGB_LED_Violet();
	 delay_ms(100);
	 RGB_LED_Off();
	 delay_ms(100);
	 RGB_LED_Violet();
	 delay_ms(100);
	 RGB_LED_Off();
}

/**************************************************************************
*函  数：void WiFi_OFFON_LED(void)
*功  能：靛青色灯闪烁提示WiFi开关
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void WiFi_OFFON_LED(void)
{
	static uint8_t cnt=0,flag = 0;
  if(WiFi_LEDflag==1)//wifi开启指示灯
	{
		if(cnt++>6) 
		{
			cnt = 0;
			WiFi_LEDflag = 0;
			Run_flag = 1; //打开运行灯
			RGB_LED_Off();
		}
		else 
		{
			if(flag)
			{
				flag = 0;
				RGB_LED_Cyan();
			}
			else
			{
				flag = 1;
				RGB_LED_Off();
			}
	 }
	}
	else if(WiFi_LEDflag==2)//wifi关闭指示灯
	{
		if(cnt++>6) 
		{
			cnt = 0;
			WiFi_LEDflag = 0;
			Run_flag = 1; //打开运行灯
			RGB_LED_Off();
		}else
		{
			if(flag)
			{
				flag = 0;
				RGB_LED_Red();
			}
			else
			{
				flag = 1;
				RGB_LED_Off();
			}
	  }
	}
}

/**************************************************************************
*函  数：void BATT_Alarm_LED(void)
*功  能：低电量红灯快闪
*参  数：无
*返回值：无
*备  注: 无
**************************************************************************/
void BATT_Alarm_LED(void)
{
	static uint8_t flag = 0;
  if(BATT_LEDflag)
	{
		if(flag)
		{
			flag = 0;
		 RGB_LED_Red();
		}
		else
		{
			flag = 1;
		 RGB_LED_Off();
		}
	}
}

/**************************************************************************
*函  数：void RGB_Unlock(uint8_t N,uint8_t flag)
*功  能：飞机解锁指示彩色灯
*参  数：N:彩灯变换频率
         flag:使能变换标志
*返回值：无
*备  注: 无
**************************************************************************/
void RGB_Unlock(uint8_t N,uint8_t flag)
{
	static uint8_t cnt = 0;
	if(flag && cnt++>N)
	{ 
	 cnt = 0;
	 RGB_LED_Rand();	
	}	
}

void OneNET_LED(uint8_t color[],uint8_t num)
{
	uint8_t i;
	for(i=0;i<num;i++)
	{
		RGB_LED_Colour_Set(color[1],color[0],color[2]);
	}
}
