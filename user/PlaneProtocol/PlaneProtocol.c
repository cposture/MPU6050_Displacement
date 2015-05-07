/***************************************************************************************
 *	FileName					:	PlaneProtocol.c
 *	CopyRight					:	
 *	ModuleName					:	匿名上位机协议
 *
 *	CPU							:	STM32F103VET6
 *	RTOS						:	
 *
 *	Create Date					:	2014/4/20
 *	Author/Corportation			:	cposture
 *
 *	Abstract Description		:	
 *
 *--------------------------------Revision History--------------------------------------
 *	No	version		Date		Revised By			Item			Description
 *	1	v1.0		2014/4/20	cposture			1				创建文件
 *
 ***************************************************************************************/



/**************************************************************
*	Debug switch Section
**************************************************************/

/**************************************************************
*	Include File Section
**************************************************************/
#include "PlaneProtocol.h"
#include "USART.h"
/**************************************************************
*	Macro Define Section
**************************************************************/
/*函数功能：根据匿名最新上位机协议写的显示姿态的程序
*具体原理看匿名的讲解视频
*/
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
/**************************************************************
*	Struct Define Section
**************************************************************/

/**************************************************************
*	Prototype Declare Section
**************************************************************/

/**************************************************************
*	Global Variable Declare Section
**************************************************************/

/**************************************************************
*	File Static Variable Define Section
**************************************************************/

/**************************************************************
*	Function Define Section
**************************************************************/

/**
*	Name...........:	Data_Send_Status	
*	description....:	向匿名上位机发送欧拉角、陀螺仪和加速度计数据
*	param..........:	Pitch	:俯仰角
						Roll	:横滚角
						Yaw		:航向角
						gyro	:三轴陀螺仪数据，顺序为x,y,z
						accel	:三轴加速度数据，顺序为x,y,z
*	return.........:	
*	precondition...:	用到了printf，需要重定向printf到串口及串口的相关初始化
*	postcondition..:	
*/
void Data_Send_Status(float Pitch, float Roll, float Yaw, int16 *gyro, int16 *accel)
{
	unsigned char i = 0;
	unsigned char _cnt = 0, sum = 0;
	unsigned int _temp;
	u8 data_to_send[50];

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x01;
	data_to_send[_cnt++] = 0;

	_temp = (int)(Roll * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = 0 - (int)(Pitch * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(Yaw * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;
	//和校验
	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	//串口发送数据
	for (i = 0; i<_cnt; i++)
		printf("%c", data_to_send[i]);
}
/**
*	Name...........:	Send_Data
*	description....:	向匿名上位机发送陀螺仪和加速度计数据
*	param..........:	gyro	:三轴陀螺仪数据，顺序为x,y,z
						accel	:三轴加速度数据，顺序为x,y,z
*	return.........:
*	precondition...:	用到了printf，需要重定向printf到串口及串口的相关初始化
*	postcondition..:
*/
void Send_Data(int16 *Gyro, int16 *Accel)
{
	unsigned char i = 0;
	unsigned char _cnt = 0, sum = 0;
	//	unsigned int _temp;
	u8 data_to_send[50];

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x02;
	data_to_send[_cnt++] = 0;


	data_to_send[_cnt++] = BYTE1(Accel[0]);
	data_to_send[_cnt++] = BYTE0(Accel[0]);
	data_to_send[_cnt++] = BYTE1(Accel[1]);
	data_to_send[_cnt++] = BYTE0(Accel[1]);
	data_to_send[_cnt++] = BYTE1(Accel[2]);
	data_to_send[_cnt++] = BYTE0(Accel[2]);

	data_to_send[_cnt++] = BYTE1(Gyro[0]);
	data_to_send[_cnt++] = BYTE0(Gyro[0]);
	data_to_send[_cnt++] = BYTE1(Gyro[1]);
	data_to_send[_cnt++] = BYTE0(Gyro[1]);
	data_to_send[_cnt++] = BYTE1(Gyro[2]);
	data_to_send[_cnt++] = BYTE0(Gyro[2]);
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;

	data_to_send[3] = _cnt - 4;
	//和校验
	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	//串口发送数据
	for (i = 0; i<_cnt; i++)
		printf("%c", data_to_send[i]);
}

void ReportData(char chrType,int16 ax,int16 ay,int16 az,int16 t)
{
        char cData = 0;
        char cCRC = 0;
        putchar(0x55);     cCRC+=0x55;
        putchar(chrType);  cCRC+=chrType;       
        cData = ax;     putchar(cData);  cCRC+=cData;
        cData = ax>>8;  putchar(cData);  cCRC+=cData;
        cData = ay;     putchar(cData);  cCRC+=cData;
        cData = ay>>8;  putchar(cData);  cCRC+=cData;
        cData = az;     putchar(cData);  cCRC+=cData;
        cData = az>>8;  putchar(cData);  cCRC+=cData;        
        cData = t;      putchar(cData);  cCRC+=cData;
        cData = t>>8;   putchar(cData);  cCRC+=cData;
        putchar(cCRC);          
}
