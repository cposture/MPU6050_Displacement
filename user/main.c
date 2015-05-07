/***************************************************************************************
 *	FileName					:	main.c
 *	CopyRight					:	
 *	ModuleName					:	
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
#include "main_cfg.h"
#include "CommonType.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "usart.h"
#include "i2c.h"
#include "clock.h"
#include "math.h"
#include "stdio.h"

#if USB_SEND_DATA
#include "USB_CommProtocol.h"
#endif

#if SD_DATA_PROCESS
#include "ff.h"
#include "bsp_sdio_sdcard.h"
#include "bsp_key.h"
#endif

#if ACCEL_SHOW
#include "PlaneProtocol.h"
#endif

#include "PosSensorDataProc.h"
#include "led.h"


/**************************************************************
*	Macro Define Section
**************************************************************/
#define DEFAULT_MPU_HZ  (100)
/**************************************************************
*	Struct Define Section
**************************************************************/

/**************************************************************
*	Prototype Declare Section
**************************************************************/

/**************************************************************
*	Global Variable Declare Section
**************************************************************/
struct int_param_s int_param;
signed char gyro_orientation[9] = {1, 0, 0, 0,1, 0, 0, 0, 1};
float q[4],Pitch, Roll,Yaw;

int16 gyro[3], accel[3];
int32 vel[2][3]={0},disp[2][3] = {0},accel_ave[3],accel_res[2][3]={0};

unsigned long timestamp,time_pre;
short sensors = INV_XYZ_GYRO| INV_XYZ_ACCEL | INV_WXYZ_QUAT;
unsigned char more;
long quat[4];

int rtn_value;

int g_drift = 0;

/* debug用的变量 */
int16 i;

#if ACCEL_SHOW
int16 accel_show[3];
#endif

#if DISP_SHOW
int16 disp_show[3],vel_show[3];
#endif
	
#if USB_SEND_DATA
USBFrame_t frame;
#endif
	
#if SIGMA_FILTER_OPEN
int16 accel_pos = 0;
#endif


#if SD_DATA_PROCESS
/* 文件系统 */
FIL fnew;									/* file objects */
FATFS fs;									/* Work area (file system object) for logical drives */
FRESULT res; 
UINT bw;            					    /* File R/W count */
char textFileBuffer[100];
char path[] = "0:newfile.txt";

int16 sign = 0;
#endif

#if  SIGMA_FILTER_OPEN
int32	accel_xyz_data[3][ACC_FILTER_COUNT];
#endif
/**************************************************************
*	File Static Variable Define Section
**************************************************************/

/**************************************************************
*	Function Define Section
**************************************************************/
 
/**
 *	Name...........:	bug_Detect_Print
 *	description....:	bug打印函数
 *	param..........:	isbug : 1：有bug；0：无bug
						has_bug ： 有bug时打印的字符串
						no_bug : 无bug时打印的字符串
 *	return.........:	
 *	precondition...:
 *	postcondition..:
 */
#if DEBUG
void bug_Detect_Print(int isbug,char * has_bug,char *no_bug) 
{ 
	if(isbug)
		printf("%s",has_bug);
	else
		printf("%s",no_bug);
}
#endif

	
int main(void)
{
	clock_conf();

	/* USART1 config 115200 8-N-1 */
    USART1_Config();
    printf("\r\n 这是一个MD移植程序 \r\n");
	
	LED_GPIO_Config();
	
	#if SD_DATA_PROCESS
	/*config key*/
	Key_GPIO_Config();	
	#endif
	   
	#if USB_SEND_DATA
	LED_GPIO_Config();
	setup_USBComm();
		
	frame.X 		= 0;
	frame.Y 		= 0;
	frame.wheel 	= 0;
	frame.leftKey 	= KeyRelease;
	frame.centerKey	= KeyRelease;
	frame.rightKey 	= KeyRelease;
		
	LED1_TOGGLE;
	#endif
	
	#if SD_DATA_PROCESS
	// 文件系统
	// Sdio Interrupt Config
	NVIC_Configuration();
		
	// Register work area for each volume (Always succeeds regardless of disk status)
	f_mount(0,&fs);		
		
	// Create new file on the drive 0
	res = f_open(&fnew, path, FA_OPEN_ALWAYS | FA_WRITE );
	#endif
	
	#if DEBUG && SD_DATA_PROCESS
		bug_Detect_Print(res,"\r\n 文件系统初始化失败\r\n","\r\n 文件系统初始化成功\r\n");
	#endif
	

    ANBT_I2C_Configuration();		//IIC初始化
	
    //BUG_DETECT_PRINT(i2c_CheckDevice(0x68<<1),"\r\n 未检测到MPU6050 \r\n","\r\n 检测到MPU6050 \r\n");
    //BUG_DETECT_PRINT((result = DMP_MPU6050_DEV_CFG()),"\r\n MPU6050失败\r\n","\r\n MPU6050 \r\n");
	
	rtn_value = mpu_init(&int_param);
	#if DEBUG
		bug_Detect_Print(rtn_value,"\r\n MPU6050初始化失败\r\n","\r\n MPU6050初始化成功\r\n");
	#endif
	
	rtn_value = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	#if DEBUG
		bug_Detect_Print(rtn_value,"\r\n DMP设置传感器失败\r\n","\r\n DMP设置传感器成功\r\n");
	#endif
	
    // Push both gyro and accel data into the FIFO.
	rtn_value = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	#if DEBUG
		bug_Detect_Print(rtn_value,"\r\n 设置FIFO失败\r\n","\r\n 设置FIFO成功\r\n");
	#endif
	
	rtn_value = mpu_set_sample_rate(DEFAULT_MPU_HZ);
	#if DEBUG
		bug_Detect_Print(rtn_value,"\r\n 设置采样率失败\r\n","\r\n 设置采样率成功\r\n");
	#endif

	rtn_value = dmp_load_motion_driver_firmware();
	#if DEBUG
		bug_Detect_Print(rtn_value,"\r\n 加载固件失败\r\n","\r\n 加载成功\r\n");
	#endif

	rtn_value = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
	#if DEBUG
		bug_Detect_Print(rtn_value,"\r\n DMP设置初始方向失败\r\n","\r\n DMP设置初始方向成功\r\n");
	#endif

	rtn_value = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT  | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
	#if DEBUG
		bug_Detect_Print(rtn_value,"\r\n DMP初始化特性失败\r\n","\r\n DMP初始化特性成功\r\n");
	#endif
	
	rtn_value = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	#if DEBUG
        bug_Detect_Print(rtn_value,"\r\n 设置FIFO输出速率失败\r\n","\r\n 设置FIFO输出速率成功\r\n");
	#endif
	
    run_self_test();

    mpu_set_dmp_state(1);
    
	//delay_ms(10000);
	
	#if  SIGMA_FILTER_OPEN
	for(i = 0; i < ACC_FILTER_COUNT * 1.5; i++)
	{
		dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
		insert_AccelData(accel);
	}
	#endif
	
    while(1)
    {
		
		LED2_TOGGLE;
		
		#if SD_DATA_PROCESS
		if( Key_Scan(GPIOA,GPIO_Pin_0,1) == KEY_ON  )
			sign = 1;
		#endif
		
        dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
		
//		if(originalPlace_Drift(gyro) == 1)
//		{
//			g_drift = 1;
//		}

		
		#if  SIGMA_FILTER_OPEN
		insert_AccelData(accel);
		#endif
		
        if((sensors & INV_WXYZ_QUAT))
        {
            // DMP所得的四元数
            q[0]=quat[0] / 1073741824.0f;
            q[1]=quat[1] / 1073741824.0f;
            q[2]=quat[2] / 1073741824.0f;
            q[3]=quat[3] / 1073741824.0f;

//            // 由四元数所得的欧拉角，单位度
//            Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]) *57.3; // pitch
//            Roll = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)*57.3; // roll
//            Yaw = 	atan2(2*(q[1]*q[2] + q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3])*57.3 ;		//感觉没有价值，注掉     
		}
		
		if((sensors & INV_XYZ_ACCEL))
		{		
			// 先对加速度进行滤波
			#if AVERGE_FILTER_OPEN
			accel_Filter(accel,accel_ave);
			#endif
			
			#if SIGMA_FILTER_OPEN
			sigma_Filter(acc_xyz_data,accel_xyz_data,accel_pos,15,4);
			accel_ave[0] = accel_xyz_data[0][accel_pos];
			accel_ave[1] = accel_xyz_data[1][accel_pos];
			accel_ave[2] = accel_xyz_data[2][accel_pos];
			
			accel_pos++;
			if (accel_pos == ACC_FILTER_COUNT)
			{
				accel_pos = 0;
			}
			
			#endif		
			
            // 将基于载体坐标系的加速度值转换为参考坐标系
            accel_BConvertToN(accel_res[1],accel_ave,q);
						
			for(i=0;i<3;i++)
			{
				if(accel_res[1][i] < ACCEL_WINDOW_H && accel_res[1][i] > ACCEL_WINDOW_L)
						accel_res[1][i] = 0;
			}
						
			accel_res[1][2] -= 14890;	
			
			#if ACCEL_SHOW
            accel_show [0] = accel_res[1][0]/16384;  
            accel_show [1] = accel_res[1][1]/16384;  
            accel_show [2] = accel_res[1][2]/16384;  
			
//			printf("%d, %d, %d\n",accel_show [0],accel_show [1],accel_show [2]);
//			printf("%d, %d, %d\r\n",gyro[0],gyro[1],gyro[2]);
			
			#if UPPER_COMPUTER_NIMING
			Send_Data(gyro,accel_show);
			#endif
			
			#if UPPER_COMPUTER_MATLAB
			ReportData(0x51,accel_show [0],accel_show [1],accel_show [2],0);
			ReportData(0x52,gyro [0],gyro [1],gyro [2],0);
			#endif
			
			#endif

			position(accel_res,vel,disp);
			
			#if USB_SEND_DATA
//			if(g_drift)
//			{
//				g_drift = 0;
//				delay_ms(3);
//				
//			}
//			else
//			{
//				frame.X = (disp[0][0] - disp[1][0])*10/16384;
//				frame.Y = (disp[1][1] - disp[0][1])*10/16384;
//				sendCom_USBComm(&frame , 1000);
//				LED1_TOGGLE;
//			}
			frame.X = (disp[0][0] - disp[1][0])*10/16384;
			frame.Y = (disp[1][1] - disp[0][1])*10/16384;
			sendCom_USBComm(&frame , 1000);
			LED1_TOGGLE;
			#endif

			#if DISP_SHOW
			disp_show[0] = disp[1][0]/16384;
			disp_show[1] = disp[1][1]/16384;
			disp_show[2] = disp[1][2]/16384;
						
			vel_show[0] = vel[1][0];
			vel_show[1] = vel[1][1];
			vel_show[2] = vel[1][2];
			
			#if UPPER_COMPUTER_NIMING
			Send_Data(vel_show,disp_show);
			#endif
			
			#endif
			
			movement_End_Check(accel_res[1],vel);
			
			// 将当前加速度、速度、位移量覆盖上一量
			for(i = 0; i < 3; i++)
			{
				vel[0][i] = vel[1][i];
				disp[0][i] = disp[1][i];
				accel_res[0][i] = accel_res[1][i];
			}
			
			#if SD_DATA_PROCESS
			if(sign)
			{
//				sprintf(textFileBuffer,"=%d, %d ,%ld, %ld, %ld, %ld=\r\n",accel[0],accel[1],vel[1][0],vel[1][1],disp[1][0],disp[1][1]);
				sprintf(textFileBuffer,"=%d, %d ,%ld, %ld, %ld, %ld=\r\n",accel[0],accel[1],accel_res[1][0],accel_res[1][1],vel[1][0],vel[1][1]);
				sign = 0;
				LED3_TOGGLE;
			}
			else
			{
//				sprintf(textFileBuffer,"%d, %d ,%ld, %ld, %ld, %ld\r\n",accel[0],accel[1],vel[1][0],vel[1][1],disp[1][0],disp[1][1]);
				sprintf(textFileBuffer,"%d, %d ,%ld, %ld, %ld, %ld\r\n",accel[0],accel[1],accel_res[1][0],accel_res[1][1],vel[1][0],vel[1][1]);
			}
//			sprintf(textFileBuffer,"%d, %ld, %ld, %d, %ld, %ld\r\n",accel[0],accel_ave[0],accel_res[1][0],accel[2],accel_ave[2],accel_res[1][2]);
//			sprintf(textFileBuffer,"%d, %ld, %ld, %d, %ld, %ld, %d, %ld, %ld\r\n",accel[0],accel_ave[0],accel_res[1][0],accel[1],accel_ave[1],accel_res[1][1],accel[0],accel_ave[0],accel_res[1][0]);
			f_write(&fnew, textFileBuffer, sizeof(textFileBuffer), &bw);
			#endif

		}	
//		delay_ms(2);
		
		#if SD_DATA_PROCESS
		f_close(&fnew);
		f_open(&fnew, path, FA_OPEN_ALWAYS | FA_WRITE );
		f_lseek(&fnew,f_size(&fnew)); 
		#endif
    }
}


