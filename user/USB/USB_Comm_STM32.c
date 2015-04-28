/***************************************************************************************
 *	FileName					:	USB_Comm_STM32.C
 *	CopyRight					:	2015/04/13
 *	ModuleName					:	Driver
 *
 *	CPU							:	STM32
 *	RTOS						:	
 *
 *	Create Data					:	2015/04/13
 *	Author/Corportation			:	QGstudio/ZhuoJianHuan
 *
 *	Abstract Description		:	USB命令帧通讯协议模块实现文件
 *
 *--------------------------------Revision History--------------------------------------
 *	No	version		Data		Revised By			Item			Description
 *	1	V1.00		2015/04/13	ZhuoJianHuan		All				文件建立
 *
 ***************************************************************************************/
/**************************************************************
*	Debug switch Section
**************************************************************/
//#define D_DISP_BASE 
/**************************************************************
*	Include File Section
**************************************************************/
#include "USB_CommProtocol.h"
#include "LED.h"
/**************************************************************
*	Macro Define Section
**************************************************************/
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
 *	@name			void USB_USART_Config(void)
 *	@description	初始化命令帧USB通讯协议
 */
void USB_USART_Config(void){
	
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		
		/* config USART1 clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
		
		/* USART1 GPIO config */
		/* Configure USART1 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		/* Configure USART1 Rx (PA.10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
			
		/* USART1 mode config */
		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure); 
		USART_Cmd(USART1, ENABLE);
}
/**
 *	@name			int16_t readByte(void)
 *	@description	串口接收一个字节
 *	@return			【-1】无效数据
 *					【非-1】有效数据
 */
int16_t readByte(void){
	
	//TODO 数据有效性检查
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){
		return -1;
	}
	else{
		return (int)USART_ReceiveData(USART1);
	}
}

/**
 *	@name			void sendByte(uint8_t ch)
 *	@description	发送一个字节
 *	@param			【ch】欲发送的字节
 */
void sendByte(uint8_t ch){
	/* 发送一个字节数据到USART1 */
	USART_SendData(USART1, (uint8_t) ch);
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		
}

/**
 *	@name			void sendBuf(uint8_t *buf, uint8_t len)
 *	@description	发送多个字节
 *	@param			【buf】欲发送的缓冲
 *					【len】缓冲长度
 */
void sendBuf(uint8_t *buf, uint8_t len){
	
	while(len--){
		sendByte(*buf);
		buf++;
	}
}


/**
 *	@name			void setup_USBComm(void)
 *	@description	初始化命令帧USB通讯协议
 */
void setup_USBComm(void){
	
	int16_t temp;
	USB_USART_Config();
	while(1){
		temp = readByte();
		if(-1 != temp && 0x66 == temp){
			//TODO 应答
			sendByte(0x77);
			break;
		}
	}

#ifdef D_DISP_BASE
	turn_LED();
#endif
}

/**
 *	@name			void sendCom_USBComm(USBFrame_p frame)
 *	@descritpion	发送一个命令帧
 *	@param			frame：欲发送的帧
 *					timeOut:超时时限，以毫秒为单位
 *	@return			0x00：发送失败
 *					0x01：【非零值】发送成功
 */
uint8_t sendCom_USBComm(USBFrame_p frame, uint16_t timeOut){
	
	uint8_t result, i, j;
	uint8_t buf[5]	= {0x00};
	int16_t temp;
	
	//TODO 指针无效，快速返回
	if(!frame){
		return 0x00;
	}
	
	//TODO 左键按键检测
	if(frame->leftKey==KeyPress){
		buf[0] |= 0x01;
	}
	//TODO 右键按键检测
	if(frame->leftKey==KeyPress){
		buf[0] |= 0x02;
	}
	//TODO 中键按键检测
	if(frame->leftKey==KeyPress){
		buf[0] |= 0x04;
	}
	
	buf[1] = frame->X;
	buf[2] = frame->Y;
	buf[3] = frame->wheel;
	
	for(i=0; i<4; i++){
		
		result = 0x00;
		for(j=0; j<8; j++){
			result = ((buf[i]<<j)&0x80)^result;
		}
		buf[4] >>= 1;
		buf[4] += result;
		buf[4] >>= 1;
		buf[4] += (~result)&0x80;
	}
	
	result = 0x00;
	sendBuf(buf, 5);
	while(--timeOut){
		if(-1 != (temp = readByte())){
			if(0x88 == temp)
				break;
			else
				sendBuf(buf, 5);
		}
		//TODO 测试
		#ifdef D_DISP_BASE
			//TODO 等待直至接收到应答
			timeOut++;
		#endif
	}
	
	#ifdef D_DISP_BASE
		turn_LED();
	#endif
	return timeOut;
}


