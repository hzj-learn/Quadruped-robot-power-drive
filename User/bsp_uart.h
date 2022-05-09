#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f10x.h"
#include "stdio.h"

    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);

		
#define  DEBUG_USARTx                   USART1        
// 串口2-USART1 打开串口2所在总线的时钟 
#define  DEBUG_USART_CLK                RCC_APB2Periph_USART1//RCC_APB2Periph_USART1
#define  DEBUG_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
// 串口2-USART1 打开串口2所在总线的时钟 
#define  DEBUG_USART_BAUDRATE           9600

// USART GPIO 引脚宏定义 打开串口2 所在的IO口线时钟
#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
// USART GPIO 引脚宏定义 打开串口2 所在的IO口线时钟
    
#define  DEBUG_USART_TX_GPIO_PORT       GPIOA   
#define  DEBUG_USART_TX_GPIO_PIN        GPIO_Pin_9
#define  DEBUG_USART_RX_GPIO_PORT       GPIOA
#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_10

#define  DEBUG_USART_IRQ                USART1_IRQn
#define  DEBUG_USART_IRQHandler         USART1_IRQHandler

void DEBUG_UART_Config(void);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void _Usart_SendString( USART_TypeDef * pUSARTx, unsigned char *str,unsigned char len);
#endif  /* __BSP_UART_H */


