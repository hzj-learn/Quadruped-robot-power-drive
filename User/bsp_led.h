#ifndef __LED_H
#define	__LED_H


#include "stm32f10x.h"


/* ����LED���ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����Ĵ��뼴�ɸı���Ƶ�LED���� */
#define LED1_GPIO_PORT    	GPIOA		              /* GPIO�˿� */
#define LED1_GPIO_CLK 	    RCC_APB2Periph_GPIOA		/* GPIO�˿�ʱ�� */
#define LED1_GPIO_PIN			  GPIO_Pin_5			        

#define LED2_GPIO_PORT    	GPIOD			              /* GPIO�˿� */
#define LED2_GPIO_CLK 	    RCC_APB2Periph_GPIOD		/* GPIO�˿�ʱ�� */
#define LED2_GPIO_PIN		    GPIO_Pin_2			        

#define  EN_UART2_GPIO_CLK         RCC_APB2Periph_GPIOA
#define  EN_UART2_GPIO_PORT        GPIOA
#define  EN_UART2_GPIO_PIN         GPIO_Pin_15

#define  SHA_GPIO_CLK         RCC_APB2Periph_GPIOB
#define  SHA_GPIO_PORT        GPIOB
#define  SHA_GPIO_PIN         GPIO_Pin_15

#define  PWM_U_GPIO_CLK         RCC_APB2Periph_GPIOB
#define  PWM_U_GPIO_PORT        GPIOB
#define  PWM_U_GPIO_PIN         GPIO_Pin_15

#define  PWM_V_GPIO_CLK         RCC_APB2Periph_GPIOB
#define  PWM_V_GPIO_PORT        GPIOB
#define  PWM_V_GPIO_PIN         GPIO_Pin_15

#define  PWM_W_GPIO_CLK         RCC_APB2Periph_GPIOB
#define  PWM_W_GPIO_PORT        GPIOB
#define  PWM_W_GPIO_PIN         GPIO_Pin_15

#define  POW_W_GPIO_CLK         RCC_APB2Periph_GPIOB
#define  POW_W_GPIO_PORT        GPIOB
#define  POW_W_GPIO_PIN         GPIO_Pin_4

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  0
#define OFF 1

/* ʹ�ñ�׼�Ĺ̼������IO*/
#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_GPIO_PORT,LED1_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED1_GPIO_PORT,LED1_GPIO_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_GPIO_PORT,LED2_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED2_GPIO_PORT,LED2_GPIO_PIN)
#define   EN_UART2(a)	if (a)	\
					GPIO_SetBits(EN_UART2_GPIO_PORT,EN_UART2_GPIO_PIN);\
					else		\
					GPIO_ResetBits(EN_UART2_GPIO_PORT,EN_UART2_GPIO_PIN)



/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 //���Ϊ�ߵ�ƽ		
#define digitalLo(p,i)		 {p->BRR=i;}	 //����͵�ƽ
#define digitalToggle(p,i) {p->ODR ^=i;} //�����ת״̬


/* �������IO�ĺ� */
#define LED1_TOGGLE		 digitalToggle(LED1_GPIO_PORT,LED1_GPIO_PIN)
#define LED1_OFF		   digitalHi(LED1_GPIO_PORT,LED1_GPIO_PIN)
#define LED1_ON			   digitalLo(LED1_GPIO_PORT,LED1_GPIO_PIN)

#define LED2_TOGGLE		 digitalToggle(LED2_GPIO_PORT,LED2_GPIO_PIN)
#define LED2_OFF		   digitalHi(LED2_GPIO_PORT,LED2_GPIO_PIN)
#define LED2_ON			   digitalLo(LED2_GPIO_PORT,LED2_GPIO_PIN)

#define N_PWM_U_H		   digitalHi(PWM_U_GPIO_PORT,PWM_U_GPIO_PIN)
#define N_PWM_U_L			 digitalLo(PWM_U_GPIO_PORT,PWM_U_GPIO_PIN)

#define N_PWM_V_H		   digitalHi(PWM_V_GPIO_PORT,PWM_V_GPIO_PIN)
#define N_PWM_V_L			 digitalLo(PWM_V_GPIO_PORT,PWM_V_GPIO_PIN)

#define N_PWM_W_H		   digitalHi(PWM_W_GPIO_PORT,PWM_W_GPIO_PIN)
#define N_PWM_W_L			 digitalLo(PWM_W_GPIO_PORT,PWM_W_GPIO_PIN)

void LED_GPIO_Config(void);

#endif /* __LED_H */