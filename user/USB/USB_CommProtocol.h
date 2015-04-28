/***************************************************************************************
 *	FileName					:	USB_CommProtocol.h
 *	CopyRight					:	2015/04/13
 *	ModuleName					:	Driver
 *
 *	CPU							:	STM32
 *	RTOS						:	
 *
 *	Create Data					:	2015/04/13
 *	Author/Corportation			:	QGstudio/ZhuoJianHuan
 *
 *	Abstract Description		:	USB����֡ͨѶЭ��ģ��ͷ�ļ�
 *
 *--------------------------------Revision History--------------------------------------
 *	No	version		Data		Revised By			Item			Description
 *	1	V1.00		2015/04/13	ZhuoJianHuan		All				�ļ�����
 *
 ***************************************************************************************/
/**************************************************************
*	Multi-Include-Prevent Section
**************************************************************/
#ifndef __FN_USB_COMM_PROTOCOL_H
#define __FN_USB_COMM_PROTOCOL_H
/**************************************************************
*	Debug switch Section
**************************************************************/
/**************************************************************
*	Include File Section
**************************************************************/
#include "stm32f10x.h"
/**************************************************************
*	Macro Define Section
**************************************************************/
/**************************************************************
*	Struct Define Section
**************************************************************/
typedef enum{
	KeyPress   = 0x00,		//��������״̬
	KeyRelease = 0x01		//�����ͷ�״̬
}KeyStatue_t;
typedef struct USBFrame_{
	KeyStatue_t leftKey;	//���
	KeyStatue_t rightKey;	//�Ҽ�
	KeyStatue_t centerKey;	//�м�
	signed char wheel;		//����ƫ��ֵ
	signed char X;			//X��ƫ��ֵ
	signed char Y;			//Y��ƫ��ֵ
}USBFrame_t, *USBFrame_p;
/**************************************************************
*	Prototype Declare Section
**************************************************************/
/**
 *	@name			void setup_USBComm(void)
 *	@description	��ʼ������֡USBͨѶЭ��
 */
void setup_USBComm(void);

/**
 *	@name			void sendCom_USBComm(USBFrame_p frame)
 *	@descritpion	����һ������֡
 *	@param			frame�������͵�֡
 *					timeOut:��ʱʱ�ޣ��Ժ���Ϊ��λ
 *	@return			0x00������ʧ��
 *					0x01��������ֵ�����ͳɹ�
 */
uint8_t sendCom_USBComm(USBFrame_p frame, uint16_t timeOut);

/**************************************************************
*	End-Multi-Include-Prevent Section
**************************************************************/
#endif

