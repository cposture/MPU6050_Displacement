/***************************************************************************************
 *	FileName					:	PlaneProtocol.h
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
void Data_Send_Status(float Pitch, float Roll, float Yaw, int16 *gyro, int16 *accel);
void Send_Data(int16 *Gyro, int16 *Accel);
/**************************************************************
*	End-Multi-Include-Prevent Section
**************************************************************/
#endif