#include "bsp_basetime.h"


/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 *typedef struct
 *{ TIM_Prescaler            ����
 *	TIM_CounterMode			     TIMx,x[6,7]û�У���������
 *  TIM_Period               ����
 *  TIM_ClockDivision        TIMx,x[6,7]û�У���������
 *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]����
 *}TIM_TimeBaseInitTypeDef; 
 *-----------------------------------------------------------------------------
 */


void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //????

    //???TIM3???
    TIM_TimeBaseStructure.TIM_Period = arr; //???????????????????????????   
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //??????TIMx???????????
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //??????:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM??????
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //??????????TIMx???????

    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //?????TIM3??,??????

    //?????NVIC??
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3??
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //?????0?
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //????3?
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ?????
    NVIC_Init(&NVIC_InitStructure);  //???NVIC???


    TIM_Cmd(TIM3, ENABLE);  //??TIMx 
}
void TIM4_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //????

    //???TIM3???
    TIM_TimeBaseStructure.TIM_Period = arr; //???????????????????????????   
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //??????TIMx???????????
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //??????:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM??????
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //??????????TIMx???????

    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //?????TIM3??,??????

    //?????NVIC??
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM3??
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //?????0?
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //????3?
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ?????
    NVIC_Init(&NVIC_InitStructure);  //???NVIC???


    TIM_Cmd(TIM4, ENABLE);  //??TIMx 
}
void BASIC_TIM3_Init(void)
{
	TIM3_Int_Init(10000,71);
}
void BASIC_TIM4_Init(void)
{
	TIM4_Int_Init(3000,71);
}



