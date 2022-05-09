#include "Delay.h"
#include "stdint.h"
#include "stm32f10x.h"

static uint8_t fac_us=0;  //us?????
static uint16_t fac_ms=0; //ms?????


void DelayInit()
{
    //??????
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    fac_us=SystemCoreClock/8000000;  //????????1/8
    fac_ms=(u16)fac_us*1000;         
}

void DelayUs(unsigned int us)
{
    u32 temp;
    SysTick->LOAD=us*fac_us;             //????
    SysTick->VAL=0x00;                   //?????
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
    do
    {
        temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));         //??????
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;   //?????
    SysTick->VAL=0x00;                         //?????
}


void DelayMs(unsigned int ms)
{
    u32 temp;
    SysTick->LOAD=ms*fac_ms;             
    SysTick->VAL=0x00;                   
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
    do
    {
        temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));   
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;   
    SysTick->VAL=0x00;                   
}

void DelayS(unsigned int s)
{
    unsigned char i;
    for(i=0;i<s;i++)
    {
        DelayMs(1000);
    }
}
