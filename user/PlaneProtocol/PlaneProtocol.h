/***************************************************************************************
 *	FileName					:	PlaneProtocol.h
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
*	Multi-Include-Prevent Section
**************************************************************/
#ifndef PLANEPROTOCOL_H
#define PLANEPROTOCOL_H
/**************************************************************
*	Debug switch Section
**************************************************************/

/**************************************************************
*	Include File Section
**************************************************************/
#include "CommonType.h"
/**************************************************************
*	Macro Define Section
**************************************************************/

/**************************************************************
*	Struct Define Section
**************************************************************/

/**************************************************************
*	Prototype Declare Section
**************************************************************/
void Data_Send_Status(float Pitch, float Roll, float Yaw);
void Send_Data(int16 *Gyro, int16 *Accel);
void ReportData(char chrType,int16 ax,int16 ay,int16 az,int16 t);
/**************************************************************
*	End-Multi-Include-Prevent Section
**************************************************************/
#endif
