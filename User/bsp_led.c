
// bsp  board support package 板级支持包

#include "bsp_led.h"
void LED_GPIO_Config(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(LED1_GPIO_CLK | LED2_GPIO_CLK|EN_UART2_GPIO_CLK |PWM_U_GPIO_CLK|PWM_V_GPIO_CLK|PWM_W_GPIO_CLK, ENABLE);
	
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;	
		GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);	

		GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;
		GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);	
	
	  GPIO_InitStructure.GPIO_Pin = EN_UART2_GPIO_PIN;
	  GPIO_Init(EN_UART2_GPIO_PORT, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = PWM_U_GPIO_PIN;
	  GPIO_Init(PWM_U_GPIO_PORT, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = PWM_V_GPIO_PIN;
	  GPIO_Init(PWM_V_GPIO_PORT, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = PWM_W_GPIO_PIN;
	  GPIO_Init(PWM_W_GPIO_PORT, &GPIO_InitStructure);
	
		GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
		GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		
		GPIO_ResetBits(PWM_U_GPIO_PORT, PWM_U_GPIO_PIN);
		GPIO_ResetBits(PWM_V_GPIO_PORT, PWM_V_GPIO_PIN);
		GPIO_ResetBits(PWM_W_GPIO_PORT, PWM_W_GPIO_PIN);
		
	  GPIO_SetBits(EN_UART2_GPIO_PORT, EN_UART2_GPIO_PIN);
}


