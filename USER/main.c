
#include "main.h"
int16_t GYRO[3];
int16_t ACC[3];
float Temperature;
FLOAT_ANGLE Angle;

#define M_PI_F 3.1416f
#include "math.h"
int main(void)
{
	uint8_t ID = 0;
	NVIV_Config();
	LED_Init();
	Delay_Init();
	//Usart1_Init(460800);
	Usart1_Init(115200); //调试用
	Usart2_Init(921600);
	IIC_Init();
	TIM4_Init();
	Exti_Init();
	
	IIC_ReadByteFromSlave(0x69,0x75,&ID);
	printf("%c\r\n",ID);
	
	MPU9250_Init();
	printf("%c\r\n",ID);
	while(1)
	{
		if(BREATH_Scan)//1000Hz
		{
			BREATH_Scan = 0;
		//	LED_Breath_User();	
			
		}
		if(IMU_Scan) //100Hz
		{
			IMU_Scan  = 0;
			Prepare_Data();
			IMUupdate_nine(&Gyr_rad,&Acc_filt,&Mag_filt,&Angle);
			//IMUupdate(&Gyr_rad,&Acc_filt,&Angle);
//			printf("yaw:%.2f  pitch:%.2f  roll:%.2f\r\n",Angle.yaw, Angle.pit, Angle.rol);
		}
		if(LED_Scan) //10Hz
		{
			LED_Scan = 0;
			RGB_LED_Runing();
			
			//RGB_LED_Rand();
			//ANO_DT_Data_Exchange(); //更新数据到上位机
		}
		if(Batt_Scan) //2.5Hz
		{
			Batt_Scan = 0;
			
		}
	}
}
#if 0 
int main(void)
{
	NVIV_Config();
	LED_Init();
	Delay_Init();
	Usart1_Init(460800);
	Usart2_Init(921600);
	IIC_Init();
	TIM4_Init();
	Exti_Init(); //外部中断初始化
	NRF24l01_Init(); //NRF初始化（红）
	MPU9250_Init(); //MPU9250初始化（绿） 
	spl06_init();
//	ICM_PowerOn();
//	FBM320_Init(); //FBM320初始化(气压计蓝) 
	MOTOR_Init(); //电机输出初始 	化
	BATT_Init(); //电池电压检测初始化
	WiFi_Switch(ENABLE); //WiFi模块开关
	OpenMV_Switch(DISABLE); //OpenMV模块开关
	PID_ReadFlash(); //Flash中的数据读取
	PidParameter_init(); //PID初始化
	RGB_LED_Off();
	while(1)
	{
		if(ANO_Scan) //500Hz
		{
			ANO_Scan = 0;
			ANO_DT_Data_Exchange(); //更新数据到上位机
		}
		if(IMU_Scan) //100Hz
		{
			IMU_Scan  = 0;
			Prepare_Data(); //获取姿态解算所需数据
			IMUupdate(&Gyr_rad,&Acc_filt,&Att_Angle); //四元数姿态解算
			Control(&Att_Angle,&Gyr_rad,&RC_Control,Airplane_Enable); //姿态控制
			RunTimer_Test();
			spl06_update();
			altitude_get();
		}
		if(LED_Scan) //10Hz
		{
			LED_Scan = 0;
			LED_Run();
			if(!Airplane_Enable&&Run_flag&&!WiFi_LEDflag)
			{
				RGB_LED_Runing(); //飞机上锁状态灯
			}
			WiFi_OFFON_LED(); //WiFi开关状态灯 
			BATT_Alarm_LED(); //电池低电压报警  
		}
		if(IRQ_Scan) //5Hz
		{
			IRQ_Scan = 0;
			NRF_SingalCheck(); //NRF通信检测
			SendToRemote(); //发送数据给遥控器
		}
		if(Batt_Scan) //2.5Hz
		{
			Batt_Scan = 0;
			NRF_GetAddr(); //分配NRF地址
			LowVoltage_Alarm();
		}
	}
}
#endif
