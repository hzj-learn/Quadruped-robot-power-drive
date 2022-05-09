#ifndef __BSP_EXTI_KEY_H
#define __BSP_EXTI_KEY_H

#include "stm32f10x.h"

#define  HU_GPIO_CLK         RCC_APB2Periph_GPIOA
#define  HU_GPIO_PORT        GPIOA
#define  HU_GPIO_PIN         GPIO_Pin_6
#define  HU_GPIO_PORTSOURCE  GPIO_PortSourceGPIOA
#define  HU_GPIO_PINSOURCE   GPIO_PinSource6
#define  HU_LINE             EXTI_Line6
#define  HU_IRQN             EXTI9_5_IRQn
#define  HU_IRQHANDLER       EXTI9_5_IRQHandler

#define  HV_GPIO_CLK         RCC_APB2Periph_GPIOA
#define  HV_GPIO_PORT        GPIOA
#define  HV_GPIO_PIN         GPIO_Pin_7
#define  HV_GPIO_PORTSOURCE  GPIO_PortSourceGPIOA
#define  HV_GPIO_PINSOURCE   GPIO_PinSource7
#define  HV_LINE             EXTI_Line7
#define  HV_IRQN             EXTI9_5_IRQn
#define  HU_IRQHANDLER       EXTI9_5_IRQHandler

#define  HW_GPIO_CLK         RCC_APB2Periph_GPIOB
#define  HW_GPIO_PORT        GPIOB
#define  HW_GPIO_PIN         GPIO_Pin_0
#define  HW_GPIO_PORTSOURCE  GPIO_PortSourceGPIOB
#define  HW_GPIO_PINSOURCE   GPIO_PinSource0
#define  HW_LINE             EXTI_Line0
#define  HW_IRQN             EXTI0_IRQn
#define  HW_IRQHANDLER       EXTI0_IRQHandler



void EXTI_KEY_Config(void);
#endif /* __BSP_EXTI_KEY_H */


