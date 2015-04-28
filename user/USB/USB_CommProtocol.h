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
 *	Abstract Description		:	USB命令帧通讯协议模块头文件
 *
 *--------------------------------Revision History--------------------------------------
 *	No	version		Data		Revised By			Item			Description
 *	1	V1.00		2015/04/13	ZhuoJianHuan		All				文件建立
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
	KeyPress   = 0x00,		//按键按下状态
	KeyRelease = 0x01		//按键释放状态
}KeyStatue_t;
typedef struct USBFrame_{
	KeyStatue_t leftKey;	//左键
	KeyStatue_t rightKey;	//右键
	KeyStatue_t centerKey;	//中键
	signed char wheel;		//滚轮偏移值
	signed char X;			//X轴偏移值
	signed char Y;			//Y轴偏移值
}USBFrame_t, *USBFrame_p;
/**************************************************************
*	Prototype Declare Section
**************************************************************/
/**
 *	@name			void setup_USBComm(void)
 *	@description	初始化命令帧USB通讯协议
 */
void setup_USBComm(void);

/**
 *	@name			void sendCom_USBComm(USBFrame_p frame)
 *	@descritpion	发送一个命令帧
 *	@param			frame：欲发送的帧
 *					timeOut:超时时限，以毫秒为单位
 *	@return			0x00：发送失败
 *					0x01：【非零值】发送成功
 */
uint8_t sendCom_USBComm(USBFrame_p frame, uint16_t timeOut);

/**************************************************************
*	End-Multi-Include-Prevent Section
**************************************************************/
#endif

