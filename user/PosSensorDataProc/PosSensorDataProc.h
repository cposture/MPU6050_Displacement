/***************************************************************************************
 *	FileName					:	PosSensorDataProc.h
 *	CopyRight					:	
 *	ModuleName					:	位移数据处理
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
#ifndef POSSENSORDATAPROC_H
#define POSSENSORDATAPROC_H
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
#define ACCEL_WINDOW_H	400
#define ACCEL_WINDOW_L	-400

#define	ACC_FILTER_COUNT	50
#define TRESHOLD_COUNT 10

#define SIGMA_FILTER_OPEN 	1
#define AVERGE_FILTER_OPEN 	0

#define GYRO_DRIFT_H	300
#define GYRO_DRIFT_L	-300

#if  SIGMA_FILTER_OPEN
extern int16	acc_xyz_data[3][ACC_FILTER_COUNT];
#endif
/**************************************************************
*	Struct Define Section
**************************************************************/

/**************************************************************
*	Prototype Declare Section
**************************************************************/
void accel_BConvertToN(int32 accel_res[3], int32 accel[3], float q[4]);
void accel_Filter(int16 accel[3], int32 acc_ave[3]);
void movement_End_Check(int32 accel_n[3], int32 vel[2][3]);
void position(int32 accel_n[2][3], int32 vel[2][3], int32 displayment[2][3]);
void sigma_Filter(int16 accel[][ACC_FILTER_COUNT], int32 accel_res[][ACC_FILTER_COUNT], int16 pos, int16 N, int16 K);
void insert_AccelData(int16 accel[3]);
int16 originalPlace_Drift(int16 gyro[3]);
/**************************************************************
*	End-Multi-Include-Prevent Section
**************************************************************/
#endif
