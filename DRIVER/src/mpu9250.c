
#include "MPU9250.h"
#include "iic_moni.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "stdio.h"
#include "structconfig.h"
#include "paramsave.h"
#include "ICM20948.h"


#define MPU9250_Temp_Sensitivity 340.00f		//LSB/℃  333.87f
#define MPU9250_RoomTemp_Offset 0						//LSB

INT16_XYZ MAG_ASA;
static int16_t 		MAG_buff[3];
static uint8_t    MPU9250_buff[14];                  //加速度 温度 陀螺仪  原始数据
INT16_XYZ	 GYRO_OFFSET_RAW,ACC_OFFSET_RAW,MAG_OFFSET_RAW;			     //零漂数据
INT16_XYZ	 MPU9250_ACC_RAW,MPU9250_GYRO_RAW,MPU9250_MAG_RAW;	     	 //读取值原始数据
uint8_t    SENSER_OFFSET_FLAG;                       //传感器校准标志位

/*****************************************************************************
*函  数：uint8_t MPU9250_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
*功  能：写一个字节数据到 MPU9250 寄存器
*参  数：reg： 寄存器地址
*        data: 要写入的数据
*返回值：0成功 1失败
*备  注：MPU9250代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU9250_WriteByte(uint8_t reg,uint8_t data)
{
	if(IIC_WriteByteToSlave(MPU9250Addr,reg,data))
	   return 1;
	else
	   return 0;
}
/*****************************************************************************
*函  数：uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf)
*功  能：从指定MPU9250寄存器读取一个字节数据
*参  数：reg： 寄存器地址
*        buf:  读取数据存放的地址
*返回值：1失败 0成功
*备  注：MPU9250代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(MPU9250Addr,reg,buf))
	   return 1;
	else
	   return 0;
}

/*****************************************************************************
*函  数：uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
*功  能：从指定寄存器写入指定长度数据
*参  数：reg：寄存器地址
*        len：写入数据长度 
*        buf: 写入数据存放的地址
*返回值：0成功 1失败
*备  注：MPU9250代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_WriteMultByteToSlave(MPU9250Addr,reg,len,buf))
	   return 1;
	else
	   return 0;
}

/*****************************************************************************
*函  数：uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
*功  能：从指定寄存器读取指定长度数据
*参  数：reg：寄存器地址
*        len：读取数据长度 
*        buf: 读取数据存放的地址
*返回值：0成功 0失败
*备  注：MPU9250代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(MPU9250Addr,reg,len,buf))
	   return 1;
	else
	   return 0;
}

/*============================以上代码移植时需要修改=========================*/

/******************************************************************************
*函  数：uint8_tMPU9250_getDeviceID(void)
*功  能：读取  MPU9250 WHO_AM_I 标识将返回 0x68
*参  数：无
*返回值：返回读取数据
*备  注：无
*******************************************************************************/
uint8_t MPU9250_getDeviceID(void)
{
    uint8_t buf;
	  MPU9250_ReadByte(MPU9250_RA_WHO_AM_I, &buf);
    return buf;
}

/******************************************************************************
*函  数：uint8_tMPU9250_testConnection(void)
*功  能：检测MPU9250 是否已经连接
*参  数：无
*返回值：1已连接 0未链接
*备  注：无
*******************************************************************************/
uint8_t MPU9250_testConnection(void) 
{
   if(MPU9250_getDeviceID() == 0x71)  
   return 1;
   else 
	 return 0;
}

/******************************************************************************
*函  数：void MPU9250_Check()
*功  能：检测IIC总线上的MPU9250是否存在
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
 void MPU9250_Check(void) 
{ 
	while(!MPU9250_testConnection())
	{
		printf("\rMPU9250 no connect...\r\n");
		RGB_LED_green(); //绿灯常亮
	}
} 
/*****************************************************************************
*函  数：uint8_t AK8963_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
*功  能：写一个字节数据到 AK8963 寄存器
*参  数：reg： 寄存器地址
*        data: 要写入的数据
*返回值：0成功 1失败
*备  注：AK8963代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t  AK8963_WriteByte(uint8_t reg,uint8_t data)
{
	if(IIC_WriteByteToSlave(AK8963Addr,reg,data))
	   return 1;
	else
	   return 0;
}
/*****************************************************************************
*函  数：uint8_t AK8963_ReadByte(uint8_t reg,uint8_t *buf)
*功  能：从指定AK8963寄存器读取一个字节数据
*参  数：reg： 寄存器地址
*        buf:  读取数据存放的地址
*返回值：1失败 0成功
*备  注：AK8963代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t AK8963_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(AK8963Addr,reg,buf))
	   return 1;
	else
	   return 0;
}

/*****************************************************************************
*函  数：uint8_t AK8963_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
*功  能：从指定寄存器写入指定长度数据
*参  数：reg：寄存器地址
*        len：写入数据长度 
*        buf: 写入数据存放的地址
*返回值：0成功 1失败
*备  注：AK8963代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t AK8963_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_WriteMultByteToSlave(AK8963Addr,reg,len,buf))
	   return 1;
	else
	   return 0;
}

/*****************************************************************************
*函  数：uint8_t AK8963_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
*功  能：从指定寄存器读取指定长度数据
*参  数：reg：寄存器地址
*        len：读取数据长度 
*        buf: 读取数据存放的地址
*返回值：0成功 0失败
*备  注：AK8963代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t AK8963_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(AK8963Addr,reg,len,buf))
	   return 1;
	else
	   return 0;
}

/*============================以上代码移植时需要修改=========================*/

/******************************************************************************
*函  数：uint8_t AK8963_getDeviceID(void)
*功  能：读取  AK8963 WIA 标识将返回 0x48
*参  数：无
*返回值：返回读取数据
*备  注：无
*******************************************************************************/
uint8_t AK8963_getDeviceID(void)
{
    uint8_t buf;
	  AK8963_ReadByte(0x00, &buf);
		printf("\rAK8963 ID:%d\r\n",buf);
    return buf;
}

/******************************************************************************
*函  数：uint8_t AK8963_testConnection(void)
*功  能：检测AK8963 是否已经连接
*参  数：无
*返回值：1已连接 0未链接
*备  注：无
*******************************************************************************/
uint8_t AK8963_testConnection(void) 
{
   if(AK8963_getDeviceID() == 0x48)  
   return 1;
   else 
	 return 0;
}
/******************************************************************************
*函  数：void AK8963_MagneReadASA(void)
*功  能：读取加速度的原始数据
*参  数：*magneData 原始数据的指针
*返回值：无
*备  注：无
*******************************************************************************/
void AK8963_MagneReadASA(void)
{
    uint8_t buf[6];
		AK8963_ReadMultBytes(0x10,3,buf);
    MAG_ASA.X = (int16_t)buf[0];
    MAG_ASA.Y = (int16_t)buf[1];
    MAG_ASA.Z = (int16_t)buf[2];
	
		printf("\rASA.X:%d ASA.Y:%d ASA.Z:%d\r\n", MAG_ASA.X, MAG_ASA.Y, MAG_ASA.Z);
}
/******************************************************************************
*函  数：void AK8963_Check()
*功  能：检测IIC总线上的AK8963是否存在
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
 void AK8963_Check(void) 
{ 
	while(!AK8963_testConnection())
	{
		printf("\rAK8963 no connect...\r\n");
		RGB_LED_green(); //绿灯常亮
	}
	AK8963_WriteByte(0x0A,0x00);
	delay_ms(10);
	AK8963_WriteByte(0x0A,0x0F);
	AK8963_MagneReadASA();
	delay_ms(10);
	AK8963_WriteByte(0x0A,0x01);
} 

	
/******************************************************************************
*函  数：void AK8963_AccRead(int16_t *magneData)
*功  能：读取加速度的原始数据
*参  数：*magneData 原始数据的指针
*返回值：无
*备  注：无
*******************************************************************************/
void AK8963_MagneRead(void)
{
    uint8_t buf[6];
		uint16_t buf2[3];
		uint8_t ready;
		AK8963_WriteByte(0x0A,0x01);
		do
		{
			AK8963_ReadByte(0x02,&ready);
		}while((ready & 0x01) == 0);
		AK8963_ReadByte(0x09,&ready);
		if((ready & 0x08) != 0)
		{
			return;
		}
		//delay_ms(10);
   	AK8963_ReadMultBytes(0x03,6,buf);
		buf2[0] = ((buf[0] << 8) | buf[1]);
		buf2[1] = ((buf[2] << 8) | buf[3]);
		buf2[2] = ((buf[4] << 8) | buf[5]);
//		printf("MAG_buf2 Value :X=%d  Y=%d  Z=%d\n",buf2[0],buf2[1],buf2[2]);
		if(buf2[0] >= 0x01 && buf2[0] <= 0x7FF8)
		{
			MAG_buff[0] = (int16_t)buf2[0];
		}
		else if(buf2[0] >= 0x8008 && buf2[0] <= 0xFFFF)
		{
			//MAG_buff[0] = -1 * (int16_t)(0xFFFF - buf2[0] + 0x0001);
			buf2[0] = (~buf2[0]) + 0x0001;
			MAG_buff[0] = -1 * (int16_t)buf2[0];
		}
		
		if(buf2[1] >= 0x01 && buf2[1] <= 0x7FF8)
		{
			MAG_buff[1] = (int16_t)buf2[1];
		}
		else if(buf2[1] >= 0x8008 && buf2[1] <= 0xFFFF)
		{
			//MAG_buff[1] = -1 * (int16_t)(0xFFFF - buf2[1] + 0x0001);
			buf2[1] = (~buf2[1]) + 0x0001;
			MAG_buff[1] = -1 * (int16_t)buf2[1];
		}
		
		if(buf2[2] >= 0x01 && buf2[2] <= 0x7FF8)
		{
			MAG_buff[2] = (int16_t)buf2[2];
		}
		else if(buf2[2] >= 0x8008 && buf2[2] <= 0xFFFF)
		{
			//MAG_buff[2] = -1 * (int16_t)(0xFFFF - buf2[2] + 0x0001);
			buf2[2] = (~buf2[2]) + 0x0001;
			MAG_buff[2] = -1 * (int16_t)buf2[2];
		}
		
		MAG_buff[0] = (int16_t)((float)MAG_buff[0] * (((float)MAG_ASA.X - 128)/2/128 + 1));
		MAG_buff[1] = (int16_t)((float)MAG_buff[1] * (((float)MAG_ASA.Y - 128)/2/128 + 1));
		MAG_buff[2] = (int16_t)((float)MAG_buff[2] * (((float)MAG_ASA.Z - 128)/2/128 + 1));
//		printf("MAG_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",MAG_buff[0],MAG_buff[1],MAG_buff[2]);
		/*
    MAG_buff[0] = (int16_t)((buf[0] << 8) | buf[1]);
		MAG_buff[0] = MAG_buff[0] * ((MAG_ASA.X - 128)/2/128 + 1);
    MAG_buff[1] = (int16_t)((buf[2] << 8) | buf[3]);
		MAG_buff[1] = MAG_buff[1] * ((MAG_ASA.Y - 128)/2/128 + 1);
    MAG_buff[2] = (int16_t)((buf[4] << 8) | buf[5]);
		MAG_buff[2] = MAG_buff[2] * ((MAG_ASA.Z - 128)/2/128 + 1);
		*/
		//printf("MAG_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",MAG_buff[0],MAG_buff[1],MAG_buff[2]);
		
}
/******************************************************************************
*函  数：void MPU9250_AccRead(int16_t *accData)
*功  能：读取加速度的原始数据
*参  数：*accData 原始数据的指针
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_AccRead(int16_t *accData)
{
    uint8_t buf[6];
   	MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H,6,buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

/******************************************************************************
*函  数：void MPU9250_GyroRead(int16_t *gyroData)
*功  能：读取陀螺仪的原始数据
*参  数：*gyroData 原始数据的指针
*返回值：无
*备  注：GYRO_XOUT = Gyro_Sensitivity * X_angular_rate 
			Nominal Conditions FS_SEL = 0
			Gyro_Sensitivity = 131 LSB/(º/s)
*******************************************************************************/
void MPU9250_GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
	
	  MPU9250_ReadMultBytes(MPU9250_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}

/******************************************************************************
*函  数：void MPU9250_TempRead(float *tempdata)
*功  能：温度值读取
*参  数：*tempdata 温度数据的指针
*返回值：无 6864
*备  注：
* TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
* (TEMP_degC - 21degC) * Temp_Sensitivity + RoomTemp_Offset =TEMP_OUT
*******************************************************************************/
void MPU9250_TempRead(float *tempdata)
{
	uint8_t buf[2];
	short data;
	MPU9250_ReadMultBytes(MPU9250_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	//*tempdata = 36.53f + ((float)data/340.0f);
	*tempdata = 21.0f + (((float) data - MPU9250_RoomTemp_Offset)/ MPU9250_Temp_Sensitivity);
}

/******************************************************************************
*函  数：void MPU9250_Init(void)
*功  能：初始化MPU9250进入工作状态
*参  数：无
*返回值：无
*备  注：DLPF 最好设为采样频率的一半！！！
*******************************************************************************/
void MPU9250_Init(void)
{
    MPU9250_Check(); //检查MPU9250是否连接
	
		MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x80); 
	  //延迟外部传感器数据的阴影处理，直到接收到所有数据
		//第6位 置位进入睡眠状态
		//第5位 当第7位为0，第6位为1时，进入睡眠与运行循环的状态 根据寄存器108从加速度计采集数据
		//第4位 陀螺仪功能失效，进入低功耗模式，可以立即启用
		//第3位 掉电时，重置内置温度电压发生器与绝对温度ADC
    delay_ms(100);
    MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x01); //唤醒MPU9250，
		//第2-0位 并选择陀螺仪x轴PLL为时钟源(1-6实际MPU会自动选取内部最合适的时钟)，(0:内部20MHz振荡器)(7：睡眠状态)
		MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_2, 0x00); //开启 XYZ的加速度计 和 陀螺仪的输出
	
		MPU9250_WriteByte(MPU9250_RA_INT_ENABLE, 0x00); //禁止中断
	 
		//[3] FSYNC_INT_EN 辅助IIC中断，在MPU9250作为主机时有效
		//[4] FIFO_OVERFLOW_EN  加速度值，陀螺仪值，温度值，磁力计值。FSYNC引脚输入信号 数据缓存
		//有个叫 FIFO 计数器的家伙会负责记录 FIFO 里面的字节数量，而且可以随时读它。 这个中断功能是告诉你有新数据可以读
		//[6] WOM_EN 运动检测
	  MPU9250_WriteByte(MPU9250_RA_SMPLRT_DIV, 0x00); //采样分频  陀螺仪和加速度计输出频率 = 1kHz（采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
		// 陀螺仪和加速度计输出频率需要大于姿态解算的速度 
		MPU9250_WriteByte(MPU9250_RA_CONFIG, MPU9250_DLPF_BW_20);//设置陀螺的输出为1kHZ,DLPF=20Hz 
		
		MPU9250_WriteByte(MPU9250_RA_GYRO_CONFIG, 0x18); //不启用XYZ的自校准，陀螺仪满量程+-2000度/秒     (最低分辨率 = 2^16/4000 = 16.4LSB/度/秒 )
	  //Fchoice_b为Fchoice的倒数  Fchoice = 11 故 [1:0]Fchoice_b = 00
		
		MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG, 0x08); //不启用XYZ的自校准  加速度满量程+-4g   (最低分辨率 = 2^16/8g = 8196LSB/g )
		MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG_2,0x04); //设置加速度计的 输出为1kHZ，DLPF=20Hz 
		//ACCEL_FCHOICE_b为ACCEL_FCHOICE的倒数 ACCEL_FCHOICE = 1 故 [3]ACCEL_FCHOICE_b = 0
		
		MPU9250_WriteByte(MPU9250_RA_INT_PIN_CFG, 0x02);//MPU 可直接访问MPU9250辅助I2C mcu可以通过INT引脚唤醒主机  BYPASS_EN
		//置为有效后，禁用i2c主接口时，i2c_master接口引脚（ES_CL和ES_DA）将进入“旁路模式”。 
	  //如果未使能且内部I2c主接口被禁用，则引脚将由于内部上拉而浮空为高电平。
		AK8963_Check();

}

/******************************************************************************
*函  数：void MPU9250_CalOff(void)
*功  能：陀螺仪加速度校准
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_CalOff(void)
{

	 SENSER_FLAG_SET(ACC_OFFSET);//加速度校准
	 SENSER_FLAG_SET(GYRO_OFFSET);//陀螺仪校准
}

/******************************************************************************
*函  数：void MPU9250_CalOff_Acc(void)
*功  能：加速度计校准
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_CalOff_Acc(void)
{
	 SENSER_FLAG_SET(ACC_OFFSET);//加速度校准
}

/******************************************************************************
*函  数：void MPU9250_CalOff_Gyr(void)
*功  能：陀螺仪校准
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_CalOff_Gyr(void)
{
	 SENSER_FLAG_SET(GYRO_OFFSET);//陀螺仪校准
}

/******************************************************************************
*函  数：void MPU9250_Read(void)
*功  能：读取陀螺仪加速度计的原始数据
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_Read(void)
{
	MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H, 14, MPU9250_buff);// 查询法读取MPU9250的原始数据
}

/******************************************************************************
*函  数：uint8_t MPU9250_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
*功  能：MPU9250零偏校准
*参  数：value： MPU9250原始数据
*        offset：校准后的零偏值
*        sensivity：加速度计的灵敏度
*返回值：1校准完成 0校准未完成
*备  注：无
*******************************************************************************/
uint8_t MPU9250_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
{
	static int32_t tempgx=0,tempgy=0,tempgz=0; 
	static uint16_t cnt_a=0;//使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存
		if(cnt_a==0)
		{
			value.X=0;
			value.Y=0;
			value.Z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_a = 1;
			sensivity = 0;
			offset->X = 0;
			offset->Y = 0;
			offset->Z = 0;
		}
		tempgx+= 	value.X;
		tempgy+= 	value.Y; 
		tempgz+= 	value.Z-sensivity ;//加速度计校准 sensivity 等于 MPU9250初始化时设置的灵敏度值（8196LSB/g）;陀螺仪校准 sensivity = 0；
		if(cnt_a==200)               //200个数值求平均
		{
			offset->X=tempgx/cnt_a;
			offset->Y=tempgy/cnt_a;
			offset->Z=tempgz/cnt_a;
			cnt_a = 0;
			return 1;
		}
		cnt_a++;
	return 0;
}	

/******************************************************************************
*函  数：void MPU9250_DataProcess(void)
*功  能：对MPU9250进行去零偏处理
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/
void MPU9250_Offset(void)
{
	
	//加速度去零偏AD值 
	MPU9250_ACC_RAW.X =((((int16_t)MPU9250_buff[0]) << 8) | MPU9250_buff[1]) - ACC_OFFSET_RAW.X;
	MPU9250_ACC_RAW.Y =((((int16_t)MPU9250_buff[2]) << 8) | MPU9250_buff[3]) - ACC_OFFSET_RAW.Y;
	MPU9250_ACC_RAW.Z =((((int16_t)MPU9250_buff[4]) << 8) | MPU9250_buff[5]) - ACC_OFFSET_RAW.Z;
	//陀螺仪去零偏AD值 
	MPU9250_GYRO_RAW.X =((((int16_t)MPU9250_buff[8]) << 8) | MPU9250_buff[9]) - GYRO_OFFSET_RAW.X;
	MPU9250_GYRO_RAW.Y =((((int16_t)MPU9250_buff[10]) << 8) | MPU9250_buff[11]) - GYRO_OFFSET_RAW.Y;
	MPU9250_GYRO_RAW.Z =((((int16_t)MPU9250_buff[12]) << 8) | MPU9250_buff[13]) - GYRO_OFFSET_RAW.Z;
	
	//MPU9250_MAGEN_RAW.X = MAGEN_buff[0];
	//MPU9250_MAGEN_RAW.Y = MAGEN_buff[1];
	//MPU9250_MAGEN_RAW.Z = MAGEN_buff[2];
	MPU9250_MAG_RAW.X = MAG_buff[0] - MAG_OFFSET_RAW.X;
	MPU9250_MAG_RAW.Y = MAG_buff[1] - MAG_OFFSET_RAW.Y;
	MPU9250_MAG_RAW.Z = MAG_buff[2] - MAG_OFFSET_RAW.Z;
	
	if(GET_FLAG(GYRO_OFFSET)) //陀螺仪进行零偏校准
	{
		if(MPU9250_OffSet(MPU9250_GYRO_RAW,&GYRO_OFFSET_RAW,0))
		{
			
			 SENSER_FLAG_RESET(GYRO_OFFSET);
			 PID_WriteFlash(); //保存陀螺仪的零偏数据
       GYRO_Offset_LED();
		   SENSER_FLAG_SET(ACC_OFFSET);//校准加速度
			
			 printf("GYRO_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",GYRO_OFFSET_RAW.X,GYRO_OFFSET_RAW.Y,GYRO_OFFSET_RAW.Z);
//			 printf("\n");
		}
	}
	if(GET_FLAG(ACC_OFFSET)) //加速度计进行零偏校准 
	{
		if(MPU9250_OffSet(MPU9250_ACC_RAW,&ACC_OFFSET_RAW,8196))
		{
			 SENSER_FLAG_RESET(ACC_OFFSET);
			 PID_WriteFlash(); //保存加速度计的零偏数据
       ACC_Offset_LED();
			 SENSER_FLAG_SET(MAG_OFFSET);//校准磁力计
			 printf("ACC_OFFSET_RAW Value X=%d  Y=%d  Z=%d\n",ACC_OFFSET_RAW.X,ACC_OFFSET_RAW.Y,ACC_OFFSET_RAW.Z); 
//			 printf("\n");
		}
	}
	
	if(GET_FLAG(MAG_OFFSET)) //陀螺仪进行零偏校准
	{
		if(MPU9250_OffSet(MPU9250_MAG_RAW,&MAG_OFFSET_RAW,0))
		{
			
			 SENSER_FLAG_RESET(MAG_OFFSET);
			 //PID_WriteFlash(); //保存陀螺仪的零偏数据
       //GYRO_Offset_LED();
			 printf("MAGEN_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",MAG_OFFSET_RAW.X,MAG_OFFSET_RAW.Y,MAG_OFFSET_RAW.Z);
//			 printf("\n");
		}
	}
}
