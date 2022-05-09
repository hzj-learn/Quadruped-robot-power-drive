#ifndef BSP_ADVANCETIME_H
#define BSP_ADVANCETIME_H

#include "stm32f10x.h"

/************�߼���ʱ��TIM�������壬ֻ��TIM1��TIM8************/
// ��ʹ�ò�ͬ�Ķ�ʱ����ʱ�򣬶�Ӧ��GPIO�ǲ�һ���ģ����Ҫע��
// ��������ʹ�ø߼����ƶ�ʱ��TIM1

#define            ADVANCE_TIM                   TIM1
#define            ADVANCE_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define            ADVANCE_TIM_CLK               RCC_APB2Periph_TIM1
// PWM �źŵ�Ƶ�� F = TIM_CLK/{(ARR+1)*(PSC+1)}
#define            ADVANCE_TIM_PERIOD            (1800-1)
#define            ADVANCE_TIM_PSC               (2-1)
#define            GENERAL_TIM_CCR1             50
#define            GENERAL_TIM_CCR2             50
#define            GENERAL_TIM_CCR3             50

#define            ADVANCE_TIM_IRQ               TIM1_UP_IRQn
#define            ADVANCE_TIM_IRQHandler        TIM1_UP_IRQHandler

// TIM1 ����Ƚ�ͨ�� U1_41   U   PA8  SD  PLUSE 
#define            ADVANCE_TIM_CH1_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            ADVANCE_TIM_CH1_PORT          GPIOA
#define            ADVANCE_TIM_CH1_PIN           GPIO_Pin_8
// TIM1 ����Ƚ�ͨ�� U1_42   V   PA9  SD  PLUSE 
#define            ADVANCE_TIM_CH2_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            ADVANCE_TIM_CH2_PORT          GPIOA
#define            ADVANCE_TIM_CH2_PIN           GPIO_Pin_9
// TIM1 ����Ƚ�ͨ�� U1_43   W   PA10 SD  PLUSE 
#define            ADVANCE_TIM_CH3_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            ADVANCE_TIM_CH3_PORT          GPIOA
#define            ADVANCE_TIM_CH3_PIN           GPIO_Pin_10


// TIM1 ����Ƚ�ͨ���Ļ���ͨ��
#define            ADVANCE_TIM_CH1N_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            ADVANCE_TIM_CH1N_PORT          GPIOB
#define            ADVANCE_TIM_CH1N_PIN           GPIO_Pin_13

// TIM1 ����Ƚ�ͨ����ɲ��ͨ��
#define            ADVANCE_TIM_BKIN_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            ADVANCE_TIM_BKIN_PORT          GPIOB
#define            ADVANCE_TIM_BKIN_PIN           GPIO_Pin_12


#define 	EN_PWM_U TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1); TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
#define 	DS_PWM_U_L TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
#define 	DS_PWM_U_H TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_Active);

#define 	EN_PWM_V TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1); TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
#define 	DS_PWM_V_L TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);
#define 	DS_PWM_V_H TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_Active);

#define 	EN_PWM_W TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1); TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
#define 	DS_PWM_W_L TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);
#define 	DS_PWM_W_H TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_Active);
           


void ADVANCE_TIM_Init(void);

#endif  /* BSP_ADVANCETIME_H */


