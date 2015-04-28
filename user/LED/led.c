#include "led.h" 

void LED_GPIO_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����GPIOB��GPIOF������ʱ��*/
		RCC_APB2PeriphClockCmd( LED_1_RCC | LED_2_RCC | LED_3_RCC, ENABLE); 

		/*ѡ��Ҫ���Ƶ�GPIOB����*/															   
		GPIO_InitStructure.GPIO_Pin = LED_1_GPIO_PIN;	

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIOB0*/
		GPIO_Init(LED_1_GPIO, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIOF����*/															   
		GPIO_InitStructure.GPIO_Pin = LED_2_GPIO_PIN;

		/*���ÿ⺯������ʼ��GPIOF7*/
		GPIO_Init(LED_2_GPIO, &GPIO_InitStructure);
		
		/*ѡ��Ҫ���Ƶ�GPIOF����*/															   
		GPIO_InitStructure.GPIO_Pin = LED_3_GPIO_PIN;

		/*���ÿ⺯������ʼ��GPIOF7*/
		GPIO_Init(LED_3_GPIO, &GPIO_InitStructure);			  

		/* �ر�����led��	*/
		GPIO_SetBits(LED_1_GPIO, LED_1_GPIO_PIN);
		
		/* �ر�����led��	*/
		GPIO_SetBits(LED_2_GPIO, LED_2_GPIO_PIN);	 
		
		/* �ر�����led��	*/
		GPIO_SetBits(LED_3_GPIO, LED_3_GPIO_PIN);	
}
