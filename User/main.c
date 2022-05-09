#include "stm32f10x.h"
#include "./advancetime/bsp_advancetime.h"
#include "bsp_led.h"
#include "bsp_exti_key.h"
#include "bsp_uart.h"
#include "bsp_i2c_gpio.h"
#include "bsp_i2c_ee.h"
#include "bsp_adc.h"
#include "bsp_systick.h"
#include "bsp_basetime.h"
extern void CAN_GPIO_Config(void);
extern  void CAN_NVIC_Config(void);
extern void CAN_Mode_Config(void);
extern void CAN_Filter_Config(void);
extern void CAN_Config(void);
extern void CAN_SetMsg(void);
//#include "can.h"
extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];
extern unsigned char hl_u;
extern unsigned char hl_v;
extern unsigned char hl_w;
extern unsigned char hl_s;
extern unsigned char old_hl_s;
extern signed int mot_count;
extern __IO int16_t speed_duty;
extern void change_ord(void);
extern void IncPIDInit(void); 
extern int IncPIDCalc(int NextPoint); 
extern void A_IncPIDInit(void); 
extern void stop_clean_pid(void);
extern int A_IncPIDCalc(int NextPoint);
extern unsigned char hal_sta;
extern uint16_t adc_res;
extern unsigned int  p_sp_count_int_e;
extern unsigned int  p_sp_count_mod;
extern uint16_t max_adc;
extern unsigned char re_disp_max;

extern unsigned char cmd_code;
extern unsigned int  plus_count;
extern unsigned char plus_s;

extern unsigned char have_one_pack;
extern unsigned char uart_rec_buff[128];
extern unsigned int GetRevCrc_16(unsigned char * pData, int nLength);
extern void shache(unsigned char a);
unsigned char learn_st=0;
unsigned char learn_re=0;
uint32_t can_rec_id =0x1555AA0D;
float  bat_v=0;
#define stop 0
#define pause 1
#define run 2
#define learning 3
__IO uint32_t flag = 0;		 //用于标志是否接收到数据，在中断函数中赋值
CanTxMsg TxMessage;			     //发送缓冲区
CanRxMsg RxMessage;				 //接收缓冲区
typedef enum
{
  CW=0,       // 顺时钟方向
  CCW=1       // 逆时针方向
}MOTOR_DIR;

typedef enum 
{
  STOP=0,    // 停止
  RUN=1      // 运转
}MOTOR_STATE;
typedef struct
{
  __IO int          motor_speed;        // 电机转速(RPM):0..2500
  __IO MOTOR_STATE  motor_state;        // 电机旋转状态
  __IO MOTOR_DIR    motor_direction;    // 电机转动方向
  __IO uint32_t     step_counter;       // 霍尔传感器步数，用于测量电机转速
  __IO uint16_t     stalling_count;     // 停机标志，如果该值超2000，认为电机停止旋转
}MOTOR_DEVICE;
typedef struct
{
  __IO int          motor_A;        // 电机转速(RPM):0..2500
  __IO MOTOR_STATE  motor_state;        // 电机旋转状态
  __IO MOTOR_DIR    motor_direction;    // 电机转动方向
  __IO uint32_t     step_counter;       // 霍尔传感器步数，用于测量电机转速
  __IO uint16_t     stalling_count;     // 停机标志，如果该值超2000，认为电机停止旋转
}A_MOTOR_DEVICE;

extern MOTOR_DEVICE bldc_dev;
extern A_MOTOR_DEVICE A_bldc_dev;
unsigned char hal_index[6];
unsigned char nhal_index[6];
unsigned char mot_v=1;
unsigned mot_start=0;
unsigned first_ord=0;
unsigned int mot_spd=0;
signed int count1=0;
unsigned char mot_st_cr=stop;
signed int set_pos=0;
#define loc_add 0x0b
///////////////////////////////////////////////////////////////PID//////////////////////////////
/* 私有类型定义 --------------------------------------------------------------*/


//定义PID结构体
typedef struct 
{
   __IO int      SetPoint;      //设定目标 Desired Value
   __IO double   Proportion;    //比例常数 Proportional Const
   __IO double   Integral;      //积分常数 Integral Const
   __IO double   Derivative;    //微分常数 Derivative Const
   __IO int      LastError;     //Error[-1]
   __IO int      PrevError;     //Error[-2]
}PID;
extern MOTOR_DEVICE bldc_dev;
extern  PID bldc_pid;
extern  PID A_pid;
// 用于保存转换计算后的电压值 	 
float ADC_ConvertedValueLocal[NOFCHANEL];  
// 软件延时
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
void en_pwm_u(void)
{
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1); 
  TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
}
void ds_pwm_u(void)
{
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1); 
  TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
}
//U+ V- //U+ W-//V+ W-/V+ U-//W+ U-//W+ V-//
void UP_VN(void)
{
	DS_PWM_W_L;
	N_PWM_W_H;
	
	EN_PWM_U;
  N_PWM_U_H;
	
	DS_PWM_V_H;
	N_PWM_V_L;
}
void UP_WN(void)
{
	DS_PWM_V_L;
	N_PWM_V_H;
	
	EN_PWM_U;
  N_PWM_U_H;
	
	DS_PWM_W_H;
	N_PWM_W_L;
}
void VP_WN(void)
{
	DS_PWM_U_L;
	N_PWM_U_H;
	
	EN_PWM_V;
  N_PWM_V_H;
	
	DS_PWM_W_H;
	N_PWM_W_L;
}
void VP_UN(void)
{
	DS_PWM_W_L;
	N_PWM_W_H;
	
	EN_PWM_V;
  N_PWM_V_H;
	
	DS_PWM_U_H;
	N_PWM_U_L;
}
void WP_UN(void)
{
	DS_PWM_V_L;
	N_PWM_V_H;
	
	EN_PWM_W;
  N_PWM_W_H;
	
	DS_PWM_U_H;
	N_PWM_U_L;
}
void WP_VN(void)
{
	DS_PWM_U_L;
	N_PWM_U_H;
	
	EN_PWM_W;
  N_PWM_W_H;
	
	DS_PWM_V_H;
	N_PWM_V_L;
}
//////////////////////////////////
void UP_VN_WN(void)//1
{
	
	EN_PWM_U;
  N_PWM_U_H;
	
	DS_PWM_V_H;
	N_PWM_V_L;
	
	DS_PWM_W_H;
	N_PWM_W_L;
}
void UP_VN_WP(void)//2
{
	EN_PWM_U;
  N_PWM_U_H;
	
	DS_PWM_V_H;
	N_PWM_V_L;
	
	EN_PWM_W;
	N_PWM_W_H;
}
void UN_VN_WP(void)//3
{
	DS_PWM_U_H;
	N_PWM_U_L;
	
	DS_PWM_V_H;
	N_PWM_V_L;
	
	EN_PWM_W;
	N_PWM_W_H;
}
void UN_VP_WP(void)//4
{
	DS_PWM_U_H;
	N_PWM_U_L;
	
	EN_PWM_V;
	N_PWM_V_H;
	
	EN_PWM_W;
	N_PWM_W_H;
}
void UN_VP_WN(void)//5
{
	DS_PWM_U_H;
	N_PWM_U_L;
	
	EN_PWM_V;
	N_PWM_V_H;
	
	DS_PWM_W_H;
	N_PWM_W_L;
}
void UP_VP_WN(void)//6
{
	EN_PWM_U;
	N_PWM_U_H;
	
	EN_PWM_V;
	N_PWM_V_H;
	
	DS_PWM_W_H;
	N_PWM_W_L;
}

unsigned char Lean(void)
{
	unsigned char i,k;
	learn_st=0;
	learn_re=0;
	LED2(1);
	N_PWM_U_L;
	N_PWM_V_L;
	N_PWM_W_L;
	
	DS_PWM_U_L;
	DS_PWM_V_L;
	DS_PWM_W_L;
	TIM1->CCR1=99;
	TIM1->CCR2=99;
	TIM1->CCR3=99;
	stop_clean_pid();
	bldc_dev.motor_state=STOP;
  A_bldc_dev.motor_state=STOP;
	DelayMs(100);
	shache(1);
	UP_VN_WN();//1
  DelayMs(500);
	LED2(0);
	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);
	hal_index[0]=hl_s;
	learn_st=1;
	learn_re=hl_s;
	LED2(1);
	//printf("\r\n S value = %d V \r\n",hl_s);
  UP_VN_WP();//2 
  DelayMs(500);
	LED2(0);
	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);
	hal_index[1]=hl_s;
	learn_st=2;
	learn_re=hl_s;
	LED2(1);
	//printf("\r\n S value = %d V \r\n",hl_s);
	UN_VN_WP();//3
  DelayMs(500);
	LED2(0);
	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);
	hal_index[2]=hl_s;
	learn_st=3;
	learn_re=hl_s;
	LED2(1);
	//printf("\r\n S value = %d V \r\n",hl_s);
	UN_VP_WP();//4
  DelayMs(500);
	LED2(0);
	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);
	hal_index[3]=hl_s;
	learn_st=4;
	learn_re=hl_s;
	LED2(1);
	//printf("\r\n S value = %d V \r\n",hl_s);
	UN_VP_WN();//5
  DelayMs(500);
	LED2(0);
	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);
	hal_index[4]=hl_s;
	learn_st=5;
	learn_re=hl_s;
	LED2(1);
	//printf("\r\n S value = %d V \r\n",hl_s);
	UP_VP_WN();//6
  DelayMs(500);
	LED2(0);
	hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);
	hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);
	hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);
	hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);
	hal_index[5]=hl_s;
	learn_st=6;
	learn_re=hl_s;
	LED2(1);
	//printf("\r\n S value = %d V \r\n",hl_s);

	N_PWM_U_L;
	N_PWM_V_L;
	N_PWM_W_L;
	
	DS_PWM_U_L;
	DS_PWM_V_L;
	DS_PWM_W_L;
	
i=0;
for(k=0;k<6;k++)
i=hal_index[k]+i;
mot_count=0;
if(i!=21)
return 1;
return 0;
}
extern unsigned char in_code;
     void start_run()
			{
			shache(1);
			mot_st_cr=run;
      N_PWM_U_H;
	    N_PWM_V_H;
	    N_PWM_W_H;
	    DS_PWM_U_L;
	    DS_PWM_V_L;
	    DS_PWM_W_L;
			TIM1->CCR1=99;
			TIM1->CCR2=99;
			TIM1->CCR3=99;
			hl_u=GPIO_ReadInputDataBit(HU_GPIO_PORT,HU_GPIO_PIN);
			hl_v=GPIO_ReadInputDataBit(HV_GPIO_PORT,HV_GPIO_PIN);
			hl_w=GPIO_ReadInputDataBit(HW_GPIO_PORT,HW_GPIO_PIN);
			hl_s=((hl_u<<2)+(hl_v<<1)+hl_w);
			old_hl_s=hl_s;
			change_ord();
			mot_v=1;
			mot_start=1;
			////////////////////////////////////////////PID////////////////////////////////////
			stop_clean_pid();
	    bldc_dev.motor_state=RUN;
      A_bldc_dev.motor_state=RUN;
			
      bldc_dev.step_counter=0;
      bldc_dev.stalling_count=0;
			
      IncPIDInit();
			A_IncPIDInit();
			
      if((bldc_dev.motor_speed*10/60)>70)
      speed_duty=bldc_dev.motor_speed*10/60;// *10/25为转速和占空比一个转换，转速（0~2500），占空比（0~1000） gave init value
      else
      speed_duty=70;
	    ////////////////////////////////////////////////////////////////
		  }
			
void start_stop()
{
	    mot_start=0;
      stop_clean_pid();
	    bldc_dev.motor_state=RUN;
      A_bldc_dev.motor_state=RUN;
	    DelayMs(1);
	    //shache(0);
}





unsigned int timer_500ms=0;
extern unsigned char uart_snd_buff[128];
unsigned char up_data=0;
unsigned char up_num=0;
extern void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
unsigned char over_cir_t=0;
unsigned char over_cir_s=0;
float now_cir=0;
extern unsigned char rec_one_fr;
extern unsigned char uart_rec_buff1[64];
unsigned char can_rec_ang[64];

int main(void)
{
	/*硬件初始化*/
	uint16_t i;
	unsigned char t=0;
	uint8_t temp[256] = {0};
	uint8_t w_temp[256] ={1,2,3,4,9,8,7};
	ADVANCE_TIM_Init();											//高级定时器1初始化
	LED_GPIO_Config();											//LED灯初始化	
	DEBUG_UART_Config();										//串口初始化
	EXTI_KEY_Config();											//外部中断初始化
	BASIC_TIM3_Init();											//基本定时器3初始化
	BASIC_TIM4_Init();											//基本定时器4初始化
	SysTick_Delay_ms_INT(72000);						//系统滴答定时器，用于延时
	DS_PWM_V_L;														  //高级定时器1PWM输出配置					
	i2c_GPIO_Config();											//IIC初始化
  CAN_Config();														//CAN通讯初始化
	CAN_SetMsg();														//CAN通讯设置
	ADCx_Init();														//ADC初始化
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);       //打开控制器电源
	GPIO_SetBits(GPIOA,GPIO_Pin_8);  				//打开驱动板电源
	ADC_ConvertedValueLocal[0] =(float) ADC_ConvertedValue[0]/4096*3.3;		//ADC采集
	DelayMs(1000);
	TIM1->CCR1=100;
	TIM1->CCR2=100;
	TIM1->CCR3=100;
  DelayMs(100);
  while (1)
	{	
			if(rec_one_fr)
			{
				rec_one_fr=0;
				TxMessage.Data[0]=uart_rec_buff1[2];
				TxMessage.Data[1]=uart_rec_buff1[3];
				TxMessage.Data[2]=uart_rec_buff1[4];
				TxMessage.Data[3]=uart_rec_buff1[5];
				if(uart_rec_buff1[61]==0xfd)
				{
				 GPIO_SetBits(GPIOA,GPIO_Pin_8);  //开启驱动板电源
				}
				else if(uart_rec_buff1[61]==0xfe)
				{
				 GPIO_ResetBits(GPIOB,GPIO_Pin_5);//关闭控制器PC电源
				 GPIO_ResetBits(GPIOA,GPIO_Pin_8);//关闭驱动板电源
				}
				CAN_Transmit(CAN1, &TxMessage);
			}
		////////////////////////////////////////////////////////////////////////////////////////////////////	
     if(over_cir_t<100)
		 {
		 over_cir_t++;
		 bat_v +=((float) ADC_ConvertedValue[0]/2048*3.3-2.5845)/0.02;
		 }
		 else
		 {
			 bat_v=bat_v/100;
			 now_cir=bat_v;
			 bat_v=0;
			 over_cir_t=0;
			 if(now_cir>95)
			 {
			 GPIO_ResetBits(GPIOA,GPIO_Pin_8);  // 关闭驱动板电源
			 }
		 }
		 
		////////////////////////////////////////////////////////////////////////////////////////////////////
		 DelayMs(1);
		 if(flag)
		 {
			 flag=0;
			 if((RxMessage.Data[0]<15)&&(RxMessage.Data[0]>=1))
			 {
			 can_rec_ang[(RxMessage.Data[0]-1)*2]=RxMessage.Data[6];
			 can_rec_ang[(RxMessage.Data[0]-1)*2+1]=RxMessage.Data[7];
			 }
		 }
		 
		 
		 ////////////////////////////////////////////////////////////////////////////////////////////////////
		 if(timer_500ms<500)
		 {
			 timer_500ms++;
		 }
		 else
		 {
			 timer_500ms=0;
			 up_data=1;
			 up_num=0;
			 uart_snd_buff[0]=0xaa;
			 uart_snd_buff[1]=0xbb;
			 uart_snd_buff[2]=0xcc;
			 uart_snd_buff[3]=0xdd;
			 for(t=0;t<24;t++)
			 {  
				 uart_snd_buff[t+2]=can_rec_ang[t];
		   }
			 uart_snd_buff[26]=now_cir*5;
		 }
		 ////////////////////////////////////////////////////////////////////////////////////////////////////
		 if((up_data)&&(up_num<64))
		 {
			Usart_SendByte(DEBUG_USARTx,uart_snd_buff[up_num]);
			up_num++;
		 }
	  }
}

