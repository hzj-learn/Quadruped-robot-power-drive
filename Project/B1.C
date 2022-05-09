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
signed int mot_count=0;
unsigned char hal_sta=0;
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

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern volatile uint32_t isr_ms;   /* 这个变量在bsp_systick.c里面定义 */
extern unsigned int  p_sp_count_int;
void SysTick_Handler(void)
{
	if(isr_ms)
	isr_ms--;
	p_sp_count_int++;
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
	if(hl_s==hal_index[next_s])
	{
	 if(mot_v)
	 {
	 if(p_start_sp_count==0)
	 {
		p_start_sp_count=1;
		p_sp_count_int=0;
		p_sp_count_mod_s=SysTick->VAL;
	 }
	 else
	 {
		p_start_sp_count=0;
		p_sp_count_mod_e=SysTick->VAL; 
		if(p_sp_count_int)
		{
		p_sp_count_int_e=p_sp_count_int-1;
		p_sp_count_mod=(p_sp_count_mod_s+(72000-p_sp_count_mod_e)); 
		if(p_sp_count_mod>=72000)
		 {
			p_sp_count_int_e+=1;
			p_sp_count_mod=(p_sp_count_mod-72000);
		 }
	  }
		else
		{
		p_sp_count_int_e=0;
		p_sp_count_mod=(p_sp_count_mod_s-p_sp_count_mod_e);
		}
	 }
   }
	 if(mot_v==0)
	 p_start_sp_count=0;//消除瞬间反转抖动 重新开始计时 反转中出现了正向抖动
	 mot_count++;
	}
	else if(hl_s==hal_index[pre_s])//本步是反向
	{
	 if(mot_v==0)
	 {
	 if(p_start_sp_count==0)
	 {
		p_start_sp_count=1;
		p_sp_count_int=0;
		p_sp_count_mod_s=SysTick->VAL;
	 }
	 else
	 {
		p_start_sp_count=0;
		p_sp_count_mod_e=SysTick->VAL; 
		if(p_sp_count_int)
		{
		p_sp_count_int_e=p_sp_count_int-1;
		p_sp_count_mod=(p_sp_count_mod_s+(72000-p_sp_count_mod_e)); 
		if(p_sp_count_mod>=72000)
		 {
			p_sp_count_int_e+=1;
			p_sp_count_mod=(p_sp_count_mod-72000);
		 }
	  }
		else
		{
		p_sp_count_int_e=0;
		p_sp_count_mod=(p_sp_count_mod_s-p_sp_count_mod_e);
		}
	 }
   }
		if(mot_v)        //消除瞬间反转抖动 重新开始计时 正转中出现了反向抖动
		p_start_sp_count=0;
		mot_count--;
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
		LED2_TOGGLE;		
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
	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);//读取霍尔U状态
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);//读取霍尔V状态
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);//读取霍尔W状态
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);                     //把霍尔状态合成一个数字
	for(i=0;old_hl_s!=hal_index[i];i++);                 //寻找上一步电机在6步相续中的位置
	for(k=0;hl_s!=hal_index[k];k++);                     //寻找旋转后本步电机在6步相续中的位置
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
	};                         //寻找上一步电机正一步和反一步 所对应的霍尔组合数字
	
	if(hl_s==hal_index[next_s])//本步电机命中到上一步预测的正转序列
	{
	 if(mot_v)
	 {
	 if(p_start_sp_count==0)
	 {
		p_start_sp_count=1;
		p_sp_count_int=0;
		p_sp_count_mod_s=SysTick->VAL;
	 }
	 else
	 {
		p_start_sp_count=0;
		p_sp_count_mod_e=SysTick->VAL; 
		if(p_sp_count_int)
		{
		p_sp_count_int_e=p_sp_count_int-1;
		p_sp_count_mod=(p_sp_count_mod_s+(72000-p_sp_count_mod_e)); 
		if(p_sp_count_mod>=72000)
		 {
			p_sp_count_int_e+=1;
			p_sp_count_mod=(p_sp_count_mod-72000);
		 }
	  }
		else
		{
		p_sp_count_int_e=0;
		p_sp_count_mod=(p_sp_count_mod_s-p_sp_count_mod_e);
		}
	 }
   }
	 if(mot_v==0)
	 p_start_sp_count=0;//消除瞬间反转抖动 重新开始计时 反转中出现了正向抖动
	 mot_count++;
	}
	else if(hl_s==hal_index[pre_s])//本步电机命中到上一步预测的反转序列
	{
	 if(mot_v==0)//假如目前命令是反转 和本步运行 一致是反转 下面进行电机换向速度计时
	 {
	 if(p_start_sp_count==0)//假如是遇到的第一次反转结果，因为上次位置随机，因此本次开始计时
	 {
		p_start_sp_count=1;
		p_sp_count_int=0;
		p_sp_count_mod_s=SysTick->VAL;
	 }
	 else
	 {
		p_start_sp_count=0;
		p_sp_count_mod_e=SysTick->VAL; 
		if(p_sp_count_int)
		{
		p_sp_count_int_e=p_sp_count_int-1;
		p_sp_count_mod=(p_sp_count_mod_s+(72000-p_sp_count_mod_e)); 
		if(p_sp_count_mod>=72000)
		 {
			p_sp_count_int_e+=1;
			p_sp_count_mod=(p_sp_count_mod-72000);
		 }
	  }
		else
		{
		p_sp_count_int_e=0;
		p_sp_count_mod=(p_sp_count_mod_s-p_sp_count_mod_e);
		}
	 }
   }
		if(mot_v)        //假如现在命令是正转 却有反转信息 重新开始计时 正转中出现了反向抖动
		p_start_sp_count=0;
		mot_count--;
	}
	else
	hal_sta=1;
	old_hl_s=hl_s;
	if(mot_start)
  change_ord();
	if( EXTI_GetITStatus(HW_LINE) != RESET )
  {
		LED2_TOGGLE;		
	}
	EXTI_ClearITPendingBit(HW_LINE);
}
// 串口中断服务函数
void DEBUG_USART_IRQHandler(void)
{
  uint8_t ucTemp;
	if(USART_GetITStatus(DEBUG_USARTx,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData(DEBUG_USARTx);
    USART_SendData(DEBUG_USARTx,ucTemp);    
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
