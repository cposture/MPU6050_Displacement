/***************************************************************************************
 *	FileName					:	PlaneProtocol.c
 *	CopyRight					:	
 *	ModuleName					:	������λ��Э��
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
 *	1	v1.0		2014/4/20	cposture			1				�����ļ�
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
/*�������ܣ���������������λ��Э��д����ʾ��̬�ĳ���
*����ԭ�������Ľ�����Ƶ
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
*	description....:	��������λ������ŷ���ǡ������Ǻͼ��ٶȼ�����
*	param..........:	Pitch	:������
						Roll	:�����
						Yaw		:�����
						gyro	:�������������ݣ�˳��Ϊx,y,z
						accel	:������ٶ����ݣ�˳��Ϊx,y,z
*	return.........:	
*	precondition...:	�õ���printf����Ҫ�ض���printf�����ڼ����ڵ���س�ʼ��
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
	//��У��
	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	//���ڷ�������
	for (i = 0; i<_cnt; i++)
		printf("%c", data_to_send[i]);
}
/**
*	Name...........:	Send_Data
*	description....:	��������λ�����������Ǻͼ��ٶȼ�����
*	param..........:	gyro	:�������������ݣ�˳��Ϊx,y,z
						accel	:������ٶ����ݣ�˳��Ϊx,y,z
*	return.........:
*	precondition...:	�õ���printf����Ҫ�ض���printf�����ڼ����ڵ���س�ʼ��
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
	//��У��
	for (i = 0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	//���ڷ�������
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
