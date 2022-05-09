#include "bsp_exti_key.h"

static void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitTStruct;
	
	/* ��һ���������ж����ȼ��ķ��� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	/* �ڶ���������NVIC�ĳ�ʼ���ṹ�� */
	NVIC_InitTStruct.NVIC_IRQChannel = HU_IRQN;
	NVIC_InitTStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitTStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitTStruct.NVIC_IRQChannelCmd = ENABLE;
	
	/* ������������NVIC��ʼ�����������úõĽṹ���Աд���Ĵ������� */
	NVIC_Init(&NVIC_InitTStruct);	

	NVIC_InitTStruct.NVIC_IRQChannel = HV_IRQN;
	NVIC_InitTStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitTStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitTStruct.NVIC_IRQChannelCmd = ENABLE;	

	NVIC_Init(&NVIC_InitTStruct);
	NVIC_InitTStruct.NVIC_IRQChannel = HW_IRQN;
	NVIC_InitTStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitTStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitTStruct.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitTStruct);
	
}

void EXTI_KEY_Config(void)
{
	GPIO_InitTypeDef GPIO_InitTStruct;
	EXTI_InitTypeDef EXTI_InitTStruct;
	
	NVIC_Config();
	
	/* ��һ������ʼ��Ҫ���ӵ�EXTI��GPIO */
	RCC_APB2PeriphClockCmd(HU_GPIO_CLK|HV_GPIO_CLK|HW_GPIO_CLK, ENABLE);
	GPIO_InitTStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	
	GPIO_InitTStruct.GPIO_Pin = HU_GPIO_PIN;
	GPIO_Init(HU_GPIO_PORT, &GPIO_InitTStruct);
	
	GPIO_InitTStruct.GPIO_Pin = HV_GPIO_PIN;	
	GPIO_Init(HV_GPIO_PORT, &GPIO_InitTStruct);
	
	GPIO_InitTStruct.GPIO_Pin = HW_GPIO_PIN;	
	GPIO_Init(HW_GPIO_PORT, &GPIO_InitTStruct);
	
	
	/* �ڶ�������ʼ��EXTI���� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_EXTILineConfig(HU_GPIO_PORTSOURCE, HU_GPIO_PINSOURCE);	
	EXTI_InitTStruct.EXTI_Line = HU_LINE;
	EXTI_InitTStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitTStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitTStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitTStruct);
	
	GPIO_EXTILineConfig(HV_GPIO_PORTSOURCE, HV_GPIO_PINSOURCE);	
	EXTI_InitTStruct.EXTI_Line = HV_LINE;
	EXTI_InitTStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitTStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitTStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitTStruct);
	
	GPIO_EXTILineConfig(HW_GPIO_PORTSOURCE, HW_GPIO_PINSOURCE);	
	EXTI_InitTStruct.EXTI_Line = HW_LINE;
	EXTI_InitTStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitTStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitTStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitTStruct);
	

}


