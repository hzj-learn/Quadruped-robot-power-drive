/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_exti_key.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_systick.h"
extern __IO uint32_t flag;
extern  CanRxMsg RxMessage;
extern uint32_t can_rec_id;
unsigned char hl_u=0;
unsigned char hl_v=0;
unsigned char hl_w=0;
unsigned char hl_s=0;
unsigned char old_hl_s=0;
unsigned char p_start_sp_count=0;
unsigned int  p_sp_count_int=0;
unsigned int  p_sp_count_int_e=0;
unsigned int  p_sp_count_mod_s=0;
unsigned int  p_sp_count_mod_e=0;
unsigned int  p_sp_count_mod=0;
unsigned int  SP_CCR=0;
unsigned int  SA_CCR=0;
unsigned char re_disp_max=0;
unsigned char return_cm=0;
unsigned char plus_s=0;
unsigned char cmd_code=0;
unsigned int  plus_count=0;
unsigned char in_code=0;
unsigned char have_one_pack=0;
unsigned char uart_rec_buff[128]="HELLO";;
unsigned char uart_rec_buff1[64]="HELLO";
unsigned char uart_snd_buff[128]=
{
	0xaa,0xbb,
	
	0x00,0x01,
	0x00,0x02,
	0x00,0x03,
	0x00,0x04,
	0x00,0x05,
	0x00,0x06,
	0x00,0x07,
	0x00,0x08,
	0x00,0x09,
	0x00,0x0a,
	0x00,0x0b,
	0x00,0x0c,
	
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
  0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	0x00,0x00,
	
	0xcc,0xdd,
};
	
	
unsigned char count_rec_num=0;

uint16_t max_adc=0;
signed int mot_count=0;
unsigned char hal_sta=0;
unsigned char H_L_LB=0;
extern void UP_VN(void);
extern void UP_WN(void);
extern void VP_WN(void);
extern void VP_UN(void);
extern void WP_UN(void);
extern void WP_VN(void);
extern unsigned char hal_index[6];
extern unsigned char nhal_index[6];
extern unsigned mot_start;
extern unsigned char mot_v;
extern uint16_t adc_res;
extern signed int mot_count;
static uint16_t A_time_count=0;
extern signed int set_pos;
int pid_result;	
unsigned char count_fat=0;
int A_pid_result;
static uint16_t time_count=0;
extern unsigned char learn_st;
extern unsigned char learn_re;
__IO int16_t speed_duty=100; // �ٶ�ռ�ձȣ�0~1000  Ϊ1000��ռ�ձ�Ϊ100%
                              // ��ʼ��ֵ���벻С��70�������������
__IO int16_t A_speed_duty=100;


#define    NOFCHANEL										 1
#define    L_H_A                         1.5
#define    L_H                           L_H_A*62
///////////////////////////////////////////////////////////////PID//////////////////////////////
/* ˽�����Ͷ��� --------------------------------------------------------------*/
typedef enum
{
  CW=0,       // ˳ʱ�ӷ���
  CCW=1       // ��ʱ�뷽��
}MOTOR_DIR;

typedef enum 
{
  STOP=0,    // ֹͣ
  RUN=1      // ��ת
}MOTOR_STATE;

typedef struct
{
  __IO int          motor_speed;        // ���ת��(RPM):0..2500
  __IO MOTOR_STATE  motor_state;        // �����ת״̬
  __IO MOTOR_DIR    motor_direction;    // ���ת������
  __IO uint32_t     step_counter;       // �������������������ڲ������ת��
  __IO uint16_t     stalling_count;     // ͣ����־�������ֵ��2000����Ϊ���ֹͣ��ת
}MOTOR_DEVICE;
typedef struct
{
  __IO int          motor_A;        // ���ת��(RPM):0..2500
  __IO MOTOR_STATE  motor_state;        // �����ת״̬
  __IO MOTOR_DIR    motor_direction;    // ���ת������
  __IO uint32_t     step_counter;       // �������������������ڲ������ת��
  __IO uint16_t     stalling_count;     // ͣ����־�������ֵ��2000����Ϊ���ֹͣ��ת
}A_MOTOR_DEVICE;

//����PID�ṹ��
typedef struct 
{
   __IO int      SetPoint;      //�趨Ŀ�� Desired Value
   __IO double   Proportion;    //�������� Proportional Const
   __IO double   Integral;      //���ֳ��� Integral Const
   __IO double   Derivative;    //΢�ֳ��� Derivative Const
   __IO int      LastError;     //Error[-1]
   __IO int      PrevError;     //Error[-2]
}PID;

/* ˽�к궨�� ----------------------------------------------------------------*/
/*************************************/
//����PID��غ�
// �����������趨�Ե������Ӱ��ǳ���
/*************************************/
#define  P_DATA                   0.1                            //P����
#define  I_DATA                   0.02                           //I����
#define  D_DATA                   0                              //D����

#define  A_P_DATA                   0.1                            //P����
#define  A_I_DATA                   0.01                           //I����
#define  A_D_DATA                   0                              //D����


/* ˽�б��� ------------------------------------------------------------------*/
MOTOR_DEVICE bldc_dev={1000,STOP,CW,0,0};
A_MOTOR_DEVICE A_bldc_dev={5000,STOP,CW,0,0};

static PID bldc_pid;
static PID A_pid;
void stop_clean_pid(void)
{
	  time_count=0;
		pid_result=0;
		count_fat=0;
	  A_time_count=0;
		A_pid_result=0;
		A_speed_duty=0;
		count_fat=0;
}

__IO uint8_t time_over  = 0; 	//��ʱ���������

/**************PID������ʼ��********************************/
void IncPIDInit(void) 
{
  bldc_pid.LastError=0;                    //Error[-1]
  bldc_pid.PrevError=0;                    //Error[-2]
  bldc_pid.Proportion=P_DATA;              //�������� Proportional Const
  bldc_pid.Integral=I_DATA;                //���ֳ���  Integral Const
  bldc_pid.Derivative=D_DATA;              //΢�ֳ��� Derivative Const
  bldc_pid.SetPoint=bldc_dev.motor_speed;  //�趨Ŀ��Desired Value
}
void A_IncPIDInit(void) 
{
  A_pid.LastError=0;                      //Error[-1]
  A_pid.PrevError=0;                      //Error[-2]
  A_pid.Proportion=A_P_DATA;              //�������� Proportional Const
  A_pid.Integral=A_I_DATA;                //���ֳ���  Integral Const
  A_pid.Derivative=A_D_DATA;              //΢�ֳ��� Derivative Const
  A_pid.SetPoint=A_bldc_dev.motor_A;      //�趨Ŀ�����
}
/********************����ʽPID�������************************************/
int IncPIDCalc(int NextPoint) 
{
  int iError,iIncpid;                                       //��ǰ���
  iError=bldc_pid.SetPoint - NextPoint;                     //��������
  iIncpid=(bldc_pid.Proportion * iError)                    //E[k]��
              -(bldc_pid.Integral * bldc_pid.LastError);     //E[k-1]��
             // +(bldc_pid.Derivative * bldc_pid.PrevError);  //E[k-2]��
              
  bldc_pid.PrevError=bldc_pid.LastError;                    //�洢�������´μ���
  bldc_pid.LastError=iError;
  return(iIncpid);                                    //��������ֵ
}

int A_IncPIDCalc(int NextPoint) 
{
  int iError,iIncpid;                                       //��ǰ���
  iError=A_pid.SetPoint - NextPoint;                     //��������
  iIncpid=(A_pid.Proportion * iError)                    //E[k]��
              -(A_pid.Integral * A_pid.LastError)     //E[k-1]��
              +(A_pid.Derivative * A_pid.PrevError);  //E[k-2]��
              
  A_pid.PrevError=A_pid.LastError;                    //�洢�������´μ���
  A_pid.LastError=iError;
  return(iIncpid);                                    //��������ֵ
}
////////////////////////////////////////////////////////////////////////PID
void Delay_us(u32 nTimer)
{
	u32 i=0;
	for(i=0;i<nTimer;i++){
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}
}
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) 
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); 
			  if(p_sp_count_int<20)
			  p_sp_count_int++;
			 	p_sp_count_mod=0; 
		    p_sp_count_int_e=p_sp_count_int;
    }
}
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET ) 
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update); 
			  plus_s=0;
        cmd_code=0;

    }
}
void USB_LP_CAN1_RX0_IRQHandler(void)
{
   
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  /* �Ƚ��Ƿ��Ƿ��͵����ݺ�ID */ 
  if((RxMessage.ExtId==can_rec_id) && (RxMessage.IDE==CAN_ID_EXT)
     && (RxMessage.DLC==8))
  {
    flag = 1; 					       //���ճɹ�

  }
  else
  {
    flag = 0; 					   //����ʧ��
  }
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern volatile uint32_t isr_ms;   /* ���������bsp_systick.c���涨�� */
extern __IO uint16_t ADC_ConvertedValue[36];
extern unsigned int  p_sp_count_int;

uint16_t adc_temp[10];
uint32_t sum_adc=0;
uint16_t adc_res=0;
#define loc_add 0x0b
void SysTick_Handler(void)
{
uint16_t i=0;
max_adc = ADC_ConvertedValue[0]; 
for (i = 1; i < NOFCHANEL; i++)          
{                                                 
if (ADC_ConvertedValue[i] > max_adc)            
max_adc = ADC_ConvertedValue[i];         
}
//adc_res=max_adc;
if(max_adc<L_H)
{
	if(count_fat<10)
	{
	sum_adc=(sum_adc+max_adc);
	count_fat++;
	re_disp_max=0;
	}
	else
	{
  adc_res = sum_adc/10; 
	count_fat=0;
	sum_adc=0;
	re_disp_max=1;
	}
}
else
{
	adc_res=max_adc;
	H_L_LB=0;
	sum_adc=0;
	count_fat=0;
	re_disp_max=1;
}
///////////////////////////////////SPID/////////////////
 
   int temp;
   
  if(bldc_dev.motor_state==RUN)
  {
    time_count++;
    if(time_count>10) // 10ms
    {
      temp=(2000000/(p_sp_count_int_e*10000+p_sp_count_mod));
      pid_result=IncPIDCalc(temp);     // ��������
      pid_result =pid_result/6;    // *1000/6000Ϊ���ӵ�ת�ٺͶ�Ӧ���ӵ�ռ�ձ�һ��ת����ת�٣�0~6000����ռ�ձȣ�0~1000��
      if((pid_result+speed_duty)<20)
        speed_duty =20;
      else if((pid_result+speed_duty)>1000)
        speed_duty =1000;
      else
        speed_duty +=pid_result;    //����Ӧ�����ӵ�ռ�ձȼ��� �ϴ�λ�� �õ�Ŀǰʵ�ʵ�ռ�ձ�
      time_count=0;
			SP_CCR=18*speed_duty/10;
			//TIM1->CCR1=SP_CCR;
      //TIM1->CCR2=SP_CCR;
      //TIM1->CCR3=SP_CCR;
      bldc_dev.step_counter=0;
    }
  }
  else
  {
    time_count=0;
		pid_result=0;
		count_fat=0;
  }
//////////////////////////////////SPID//////////////////
///////////////////////////////////APID/////////////////
  unsigned int c_a=0;
	      int A_temp;
      
  if(A_bldc_dev.motor_state==RUN)
  {
    A_time_count++;
    if(A_time_count>=1) // 2ms
    {
      c_a=(adc_res*6600)/4096; //adc_res/4096*3.3/0.05*100
			A_temp=c_a;
      A_pid_result=A_IncPIDCalc(A_temp);     // ���뵱ǰ��õĵ���ֵ�õ������ӵĵ���ֵ
      A_pid_result =A_pid_result/6;          // *�����ӵĵ���ת�������ӵİٷֱȣ�������0~3000����ռ�ձȣ�0~1000�� ����ÿ����3��ռ�ձ�����1��
      if((A_pid_result+A_speed_duty)<20)
        A_speed_duty =20;
      else if((A_pid_result+A_speed_duty)>1000)
        A_speed_duty =1000;
      else
        A_speed_duty +=A_pid_result;    
      A_time_count=0;
			SA_CCR=18*A_speed_duty/10;           //����Ӧ�����ӵ�ռ�ձȼ��� �ϴ�λ�� �õ�Ŀǰʵ�ʵ�ռ�ձ�
			if(SA_CCR>SP_CCR)                    //�����������޶������Ƚϵõ������������ٶȻ���������Ϊ�������ٵ�ʱ�򣬵����Ƚ�С���͵�����������ֵ�ȽϺ󣬻��һ���Ƚϴ����ֵSA��ֵ���ӽ��ٷְ٣���ʱ���ٶȰ��յ��������� �Ļ����ٶȻ��ܷ�
			{                                    //2����Ϊ��
			TIM1->CCR1=SP_CCR;
      TIM1->CCR2=SP_CCR;
      TIM1->CCR3=SP_CCR;
			}
			else                                //�ٶȻ�������ڵ�������������Ϊ�������ٵ�ʱ�򣬱�����������ٶ�ƫ����ٶȻ��õ���PID�����100ռ�ձȣ���ʱ���Ե���������Ϊ�����������ٶȻ��ٷ�֮�٣���̬�£����յ�
			{
			TIM1->CCR1=SA_CCR;                  //1����Ϊ��  ����ǯ�Ӽ�סʱ�򣬵���Խ��Խ�ӽ�Ŀ���޶�ֵ������SA_CCR Խ��ԽС  ��ʱ�������·��������
      TIM1->CCR2=SA_CCR;
      TIM1->CCR3=SA_CCR;
			}
      A_bldc_dev.step_counter=0;
    }
  }
  else
  {
    A_time_count=0;
		A_pid_result=0;
		A_speed_duty=0;
		count_fat=0;
  }
//////////////////////////////////APID//////////////////
	if(isr_ms)
	isr_ms--;
}


void change_ord(void)
{
	if(mot_v)
	{
	if(hl_s==hal_index[0])
	WP_VN();
	else if(hl_s==hal_index[1])
	WP_UN();
	else if(hl_s==hal_index[2])
	VP_UN();
	else if(hl_s==hal_index[3])
	VP_WN();
	else if(hl_s==hal_index[4])
	UP_WN();
	else if(hl_s==hal_index[5])
	UP_VN();
 }
	else
	{
	if(hl_s==hal_index[3])
	WP_VN();
	else if(hl_s==hal_index[4])
	WP_UN();
	else if(hl_s==hal_index[5])
	VP_UN();
	else if(hl_s==hal_index[0])
	VP_WN();
	else if(hl_s==hal_index[1])
	UP_WN();
	else if(hl_s==hal_index[2])
	UP_VN();
 }
}

void HU_IRQHANDLER(void)
{
	unsigned char i=0;
	unsigned char k=0;
	unsigned char next_s=0;
	unsigned char pre_s=0;

	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);
	for(i=0;old_hl_s!=hal_index[i];i++);
	for(k=0;hl_s!=hal_index[k];k++);
	switch(i)
	{
		case 0:
		next_s=1;
		pre_s =5;
		break;
		case 1:
		next_s=2;
		pre_s =0;
		break;
		case 2:
		next_s=3;
		pre_s =1;
		break;
		case 3:
		next_s=4;
		pre_s =2;
		break;
		case 4:
		next_s=5;
		pre_s =3;
		break;
		case 5:
		next_s=0;
		pre_s =4;
		break;
	};
if(hl_s==hal_index[next_s])//����������е���һ��Ԥ�����ת����
	{
	 if(p_start_sp_count==0)
	 {
		p_start_sp_count=1;
		p_sp_count_int=0;
		TIM3->CNT=0;
		TIM_Cmd(TIM3, ENABLE);
		//p_sp_count_mod_s=SysTick->VAL;
	 }
	 else
	 {
		p_start_sp_count=0;
		p_sp_count_mod=TIM3->CNT; 
		p_sp_count_int_e=p_sp_count_int;
		//TIM_Cmd(TIM3, DISABLE);
	 }
	 if(return_cm)
	 mot_count++;
	 else
	 mot_count--;
	 if(set_pos==mot_count)
	 {
		 have_one_pack=1;
		 uart_rec_buff[1]=0x89;
	 }
	}
	else if(hl_s==hal_index[pre_s])//����������е���һ��Ԥ��ķ�ת����
	{
	 if(p_start_sp_count==0)
	 {
		p_start_sp_count=1;
		p_sp_count_int=0;
		TIM3->CNT=0;
		TIM_Cmd(TIM3, ENABLE);
	 }
	 else
	 {
		p_start_sp_count=0;
		p_sp_count_mod=TIM3->CNT; 
		p_sp_count_int_e=p_sp_count_int;
	 }
	 if(return_cm)
	 mot_count--;
	 else
	 mot_count++;
	 	 if(set_pos==mot_count)
	 {
		 have_one_pack=1;
		 uart_rec_buff[1]=0x89;
	 }
	}
	else
	hal_sta=1;
	old_hl_s=hl_s;
	if(mot_start)
  change_ord();
	if( EXTI_GetITStatus(HU_LINE) != RESET )
  {
		LED1_TOGGLE;		
	}
		if( EXTI_GetITStatus(HV_LINE) != RESET )
  {
		;//LED2_TOGGLE;		
	}
	EXTI_ClearITPendingBit(HV_LINE);
	EXTI_ClearITPendingBit(HU_LINE);
}

void HW_IRQHANDLER(void)
{
	unsigned char i=0;
	unsigned char k=0;
	unsigned char next_s=0;
	unsigned char pre_s=0;
	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);//��ȡ����U״̬
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);//��ȡ����V״̬
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);//��ȡ����W״̬
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);                     //�ѻ���״̬�ϳ�һ������
	for(i=0;old_hl_s!=hal_index[i];i++);                 //Ѱ����һ�������6�������е�λ��
	for(k=0;hl_s!=hal_index[k];k++);                     //Ѱ����ת�󱾲������6�������е�λ��
	switch(i)
	{
		case 0:
		next_s=1;
		pre_s =5;
		break;
		case 1:
		next_s=2;
		pre_s =0;
		break;
		case 2:
		next_s=3;
		pre_s =1;
		break;
		case 3:
		next_s=4;
		pre_s =2;
		break;
		case 4:
		next_s=5;
		pre_s =3;
		break;
		case 5:
		next_s=0;
		pre_s =4;
		break;
	};                         //Ѱ����һ�������һ���ͷ�һ�� ����Ӧ�Ļ����������
	
	if(hl_s==hal_index[next_s])//����������е���һ��Ԥ�����ת����
	{
	 if(p_start_sp_count==0)
	 {
		p_start_sp_count=1;
		p_sp_count_int=0;
		TIM3->CNT=0;
		TIM_Cmd(TIM3, ENABLE);
	 }
	 else
	 {
		p_start_sp_count=0;
		p_sp_count_mod=TIM3->CNT; 
		p_sp_count_int_e=p_sp_count_int;
	 }
	 if(return_cm)
	 mot_count++;
	 else
	 mot_count--;
	 	 if(set_pos==mot_count)
	 {
		 have_one_pack=1;
		 uart_rec_buff[1]=0x89;
	 }
	}
	else if(hl_s==hal_index[pre_s])//����������е���һ��Ԥ��ķ�ת����
	{
	 if(p_start_sp_count==0)
	 {
		p_start_sp_count=1;
		p_sp_count_int=0;
		TIM3->CNT=0;
		TIM_Cmd(TIM3, ENABLE);
	 }
	 else
	 {
		p_start_sp_count=0;
		p_sp_count_mod=TIM3->CNT; 
		p_sp_count_int_e=p_sp_count_int;
	 }
	 if(return_cm)
	 mot_count--;
	 else
	 mot_count++;
	 	if(set_pos==mot_count)
	 {
		 have_one_pack=1;
		 uart_rec_buff[1]=0x89;
	 }
	}
	else
	hal_sta=1;//������ǰ�󣬻������󣬵����޷�����
	old_hl_s=hl_s;
	if(mot_start)
  change_ord();
	if( EXTI_GetITStatus(HW_LINE) != RESET )
  {
		;//LED2_TOGGLE;		
	}
	EXTI_ClearITPendingBit(HW_LINE);
}
// �����жϷ�����
extern unsigned int GetRevCrc_16(unsigned char * pData, int nLength);
unsigned int crc_rec=9;
extern float  bat_v;
unsigned char rec_sta=0;
unsigned char rec_b0=0;
unsigned char rec_one_fr=0;
void DEBUG_USART_IRQHandler(void)
{
	unsigned char i=0;
	if(USART_GetITStatus(DEBUG_USARTx,USART_IT_RXNE)!=RESET)
	{	
		USART_ClearITPendingBit(DEBUG_USARTx, USART_IT_RXNE);
		TIM4->CNT=0;
		rec_b0 = USART_ReceiveData(DEBUG_USARTx);  
		if(count_rec_num<64)
		{
		if(rec_sta==0)
		{
		 if(rec_b0==0xaa)
	   {
   	  rec_sta=1;
			uart_rec_buff[0]=rec_b0;
		 }
		 else
		 {
		 rec_sta=0;
		 count_rec_num=0;
		 }
		}
		else if(rec_sta==1)
		{
		 if(rec_b0==0xbb)
	   {
   	  rec_sta=2;
			uart_rec_buff[1]=rec_b0;
			 count_rec_num=1;
		 }
		 else
		 {
		 rec_sta=0;
		 count_rec_num=0;
		 }
		}
		else if((rec_sta==2)&&(count_rec_num<64))
		 {
			count_rec_num++;
			 uart_rec_buff[count_rec_num]=rec_b0;
      if(count_rec_num==63)
      {
				rec_sta=0;
				count_rec_num=0;
				if((uart_rec_buff[62]==0xcc)&&(uart_rec_buff[63]==0xdd))
				{
				rec_one_fr=1;
				for(i=0;i<64;i++)
				uart_rec_buff1[i]=uart_rec_buff[i];
				}
			}
	   }
		 else
		 {
				rec_sta=0;
				count_rec_num=0;
		 }
	  }
	}	
        if(USART_GetITStatus(DEBUG_USARTx, USART_IT_TXE) != RESET)
        {   
                USART_ClearITPendingBit(DEBUG_USARTx,USART_IT_TXE);
                USART_ITConfig(DEBUG_USARTx, USART_IT_TXE, DISABLE);
//                 USART1->SR &= ~(1<<7);
        }  	
				if(USART_GetITStatus(DEBUG_USARTx, USART_IT_TC) != RESET)
				{   
                USART_ClearITPendingBit(DEBUG_USARTx,USART_IT_TC);
                USART_ITConfig(DEBUG_USARTx, USART_IT_TC, DISABLE);
//                 USART1->SR &= ~(1<<7);
        }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
