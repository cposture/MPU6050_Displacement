/***************************************************************************************
 *	FileName					:	PosSensorDataProc.c
 *	CopyRight					:	
 *	ModuleName					:	λ�����ݴ���
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
#include "math.h"
#include "PosSensorDataProc.h"
/**************************************************************
*	Macro Define Section
**************************************************************/
#define SIGMA_WIDTH			2
/**************************************************************
*	Struct Define Section
**************************************************************/

/**************************************************************
*	Prototype Declare Section
**************************************************************/

/**************************************************************
*	Global Variable Declare Section
**************************************************************/
#if AVERGE_FILTER_OPEN || SIGMA_FILTER_OPEN
int16	acc_xyz_data[3][ACC_FILTER_COUNT] = { 0 };
int16	acc_data_index = 0;
#endif
/**************************************************************
*	File Static Variable Define Section
**************************************************************/

/**************************************************************
*	Function Define Section
**************************************************************/
/**
*	Name...........:	accel_BConverttoN
*	description....:	�����ٶȵ�ֵ����������ϵתΪ��������ϵ
*	param..........:	accel_res[out]	:ת���õ���ֵ��˳��Ϊx,y,z
						accel			:��ת������������ϵ�ļ��ٶ�ֵ
						q				:��Ԫ��
*	return.........:
*	precondition...:
*	postcondition..:
*/
void accel_BConvertToN(int32 accel_res[3], int32 accel[3], float q[4])
{
	accel_res[0] = (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])	*accel[0] + 2 * (q[1] * q[2] - q[0] * q[3])	                        *accel[1] + 2 * (q[0] * q[2] + q[1] * q[3])                          *accel[2];
//	accel_res[1] = 2 * (q[0] * q[3] + q[1] * q[2])					        *accel[0] + (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3])	    *accel[1] + (-2 * q[0] * q[1] + 2 * q[2] * q[3])                         *accel[2];
	accel_res[1] = 2 * (q[0] * q[3] + q[1] * q[2])					        *accel[0] + (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3])	    *accel[1] + 2 *(q[2] * q[3] - q[0] * q[1])                         *accel[2];
	accel_res[2] = 2 * (q[1] * q[3] - q[0] * q[2])							*accel[0] + 2 * (q[0] * q[1] + q[2] * q[3])                           *accel[1] + (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])	*accel[2];
}

#if AVERGE_FILTER_OPEN
/**
*	Name...........:	accel_Filter
*	description....:	��ԭʼ���ݼ��ٶ�ֵ�����˲�
*	param..........:	accel[in,changed]	:ԭʼ���ݼ��ٶ�ֵ
						acc_ave[out]	:�˲��õ��ļ��ٶ�ֵ
*	return.........:	
*	precondition...:
*	postcondition..:
*/
void accel_Filter(int16 accel[3], int32 acc_ave[3])
{
	int i, j;
	int32	acc_data_sum[3] = { 0 };

	//�Ƚ���һ�λ�е�����˲�
	for (i = 0; i < 3; i++)
	{
		if (accel[i] < ACCEL_WINDOW_H && accel[i] > ACCEL_WINDOW_L)
			accel[i] = 0;
	}

	//��i��ļ��ٶȱ�����acc_data_index����
	for (i = 0; i<3; i++)
	{
		acc_xyz_data[i][acc_data_index] = accel[i];
	}

	//acc_data_indexѭ����1
	acc_data_index++;

	if (acc_data_index == ACC_FILTER_COUNT)
	{
		acc_data_index = 0;
	}

	//�����
	for (i = 0; i<3; i++)
	{
		for (j = 0; j<ACC_FILTER_COUNT; j++)
		{
			acc_data_sum[i] += acc_xyz_data[i][j];
		}
		acc_ave[i] = acc_data_sum[i] / ACC_FILTER_COUNT;
	}

	//�ٶ�acc_ave����һ�λ�е�����˲�
	for (i = 0; i<3; i++)
	{
		if (acc_ave[i] < ACCEL_WINDOW_H && acc_ave[i] > ACCEL_WINDOW_L)
			acc_ave[i] = 0;
	}
}
#endif
/**
*	Name...........:	movement_End_Check
*	description....:	������ٶȵ�ֵΪ0�Ĵ�����������ֵ�����ٶȵ�ֵ��0
*	param..........:	accel_n	:���ٶ�ֵָ��
						vel[in,changed]	:�ٶ�ֵָ��,vel[1][0-2]Ϊ��ǰ�ٶȣ�vel[0][0-2]Ϊ��һ�ٶ�
*	return.........:
*	precondition...:
*	postcondition..:
*/
void movement_End_Check(int32 accel_n[3], int32 vel[2][3])
{
	static unsigned int countx = 0, county = 0, countz = 0;
	//����X��
	if (accel_n[0] == 0) //we count the number of acceleration samples that equals cero
	{
		countx++;
	}
	else
	{
		countx = 0;
	}
	if (countx >= TRESHOLD_COUNT) //if this number exceeds TRESHOLD_COUNT, we can assume that velocity is cero
	{
		vel[1][0] = 0;
		vel[0][0] = 0;
	}
	//����Y��
	if (accel_n[1] == 0) //we do the same for the Y axis
	{
		county++;
	}
	else
	{
		county = 0;
	}
	if (county >= TRESHOLD_COUNT)
	{
		vel[1][1] = 0;
		vel[0][1] = 0;
	}
	//����Z��
	if (accel_n[2] == 0)
	{
		countz++;
	}
	if (countz >= TRESHOLD_COUNT)
	{
		vel[1][2] = 0;
		vel[0][2] = 0;
	}
}
/**
*	Name...........:	position
*	description....:	λ�Ƽ���
*	param..........:	accel_n	:��������ϵ�ļ��ٶ�ֵָ�룬accel_n[1][0-2]Ϊ��ǰ���ٶȣ�accel_n[0][0-2]Ϊǰһ���ٶ�
						vel[in,out]		:�ٶ�ֵָ�룬vel[1][0-2]Ϊ��ǰ�ٶȣ�vel[0][0-2]Ϊ��һ�ٶ�
						displayment[out]	:λ��ָ�룬displayment[1][0-2]Ϊ��ǰλ�ƣ�displayment[0][0-2]Ϊ��һλ��
*	return.........:
*	precondition...:
*	postcondition..:
*/
void position(int32 accel_n[2][3], int32 vel[2][3], int32 displayment[2][3])
{
	int	i;

	for (i = 0; i < 3; i++)
	{
		vel[1][i] = vel[0][i] + ((accel_n[0][i] + accel_n[1][i]) >> 1);
		displayment[1][i] = displayment[0][i] + ((vel[1][i] + vel[0][i]) >> 1);
	}
}

#if SIGMA_FILTER_OPEN
void sigma_Filter(int16 accel[][ACC_FILTER_COUNT], int32 accel_res[][ACC_FILTER_COUNT], int16 pos, int16 N, int16 K)
{
	int32 sums[3][2] = { 0 }, sum[3] = { 0 };
	int16 i, j, temp, count[3] = {0};
	double delta[3], mean;

	for (i = 0; i < 3; i++)
	{
		//��posΪ�������ģ���accel[pos-N,pos+N]�ĺ͡�ƽ����
		for (j = pos - N; j <= pos + N; j++)
		{
			temp = accel[i][(j + ACC_FILTER_COUNT) % ACC_FILTER_COUNT];
			sums[i][0] += temp;
			sums[i][1] += temp * temp;
		}
		//�����е�ƽ�����ͷ��������sigma��Χ
		mean = sums[i][0] / (2 * N + 1);
		delta[i] = sums[i][1] / (2 * N + 1) - mean * mean;
		delta[i] = SIGMA_WIDTH * sqrt(delta[i]);

		//�����sigma��Χ��Ԫ�ظ�������
		for (j = pos - N; j <= pos + N; j++)
		{
			if (accel[i][(j + ACC_FILTER_COUNT) % ACC_FILTER_COUNT] < delta[i] + accel[i][pos] && accel[i][(j + ACC_FILTER_COUNT) % ACC_FILTER_COUNT] > accel[i][pos] - delta[i])
			{
				count[i]++;
				sum[i] += accel[i][(j + ACC_FILTER_COUNT) % ACC_FILTER_COUNT];
			}
		}

		//������ֵ������sigma��Χ���Ԫ�ص�ƽ�������
		if (count[i] >= K)
		{
			accel_res[i][pos] = sum[i] / count[i];
		}
		else//������������Ԫ�ص�ƽ�������������accel[pos]
		{
			accel_res[i][pos] = (sums[i][0] - accel[i][pos]) / (2 * N);
		}
	}
}

void insert_AccelData(int16 accel[3])
{
	int i;
	
	//��i��ļ��ٶȱ�����acc_data_index����
	for (i = 0; i<3; i++)
	{
		acc_xyz_data[i][acc_data_index] = accel[i];
	}

	//acc_data_indexѭ����1
	acc_data_index++;

	if (acc_data_index == ACC_FILTER_COUNT)
	{
		acc_data_index = 0;
	}
}
#endif