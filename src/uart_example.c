/****************************************************************************
 *   $Id:: uart_example.c                                                   $
 *   Project: NXP QN9020 UART example
 *
 *   Description:
 *     This file contains UART driver usage.
 *
****************************************************************************/

#include "uart.h"
#include "system.h"

#include <stdio.h>
#include <stdlib.h>

#include "asm_prototype.h"

volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;

void led_blink_left(void)
{
    rx_flag = 0;
}

void led_blink_right(void)
{
    tx_flag = 0;
}

__IO uint32_t g_Ticks;

void SysTick_Handler(void)
{
  g_Ticks++;
}

uint32_t HAL_Ticks(void)
{
	return g_Ticks;
}

void HAL_Delay(uint32_t t)
{
	uint32_t d = t + HAL_Ticks();
	while(d > HAL_Ticks())
	{
		__NOP();
	}  
}

#define	TEST_N	10

/**
 * @brief Uart example
 */
int main (void)
{
	uint32_t tmpTick;
	
	  SysTick_Config(g_SystemClock / 1000);

#if __AHB_CLK == 32000UL
    //Initialize uart0 with 1200 baudrate, 8bit data, 1 stop bit, no parity, LSB bitorder, no HW flow control.
    uart_init(QN_UART0, __USART_CLK, UART_1200);
#else
    //Initialize uart0 with 115200 baudrate, 8bit data, 1 stop bit, no parity, LSB bitorder, no HW flow control.
    uart_init(QN_UART0, __USART_CLK, UART_115200);
    //uart_init(QN_UART0, __USART_CLK, UART_57600);
#endif
    uart_tx_enable(QN_UART0, MASK_ENABLE);
    uart_rx_enable(QN_UART0, MASK_ENABLE);

    //Initialize uart1 with 115200 baudrate, 8bit data, 1 stop bit, no parity, LSB bitorder, no HW flow control.
//     uart_init(QN_UART1, __USART_CLK, UART_115200);
//     uart_tx_enable(QN_UART1, MASK_ENABLE);
//     uart_rx_enable(QN_UART1, MASK_ENABLE);

		printf("Sys:%u Hz,AHB:%u Hz,APB:%u Hz, CPUID:%08X\n",
		g_SystemClock,
		g_AhbClock,
		g_ApbClock,
		SCB->CPUID);

		for(uint32_t i=0; i<TEST_N; ++i)
		{
			printf("%08X ", rand());
		}
		printf("\n");
		
	//Test Addition/Mulitiplication Cycles
#define	TEST_ADD_MUL_NUM	50000
	//If the muliplication takes similar cycles, it is a single cycle multiplication implementation
	tmpTick = g_Ticks;
	for(uint32_t i=0; i<TEST_ADD_MUL_NUM; ++i)
	{
		uint32_t tn = 101;
		asm_simple_add(tn, 456);
	}
	tmpTick = g_Ticks-tmpTick;
	printf("A:%u\n", tmpTick);	
	
	tmpTick = g_Ticks;
	for(uint32_t i=0; i<TEST_ADD_MUL_NUM; ++i)
	{		
		uint32_t tn = 101;
		asm_simple_mul(tn, 456);
	}
	tmpTick = g_Ticks-tmpTick;
	printf("M:%u\n", tmpTick);	
	
    while (1)                               
    {		
			HAL_Delay(2000);
			printf("%08X ", rand());
			
//        //uart0 receive data
//        rx_flag = 1;
//        uart_read(QN_UART0, buffer, 10, led_blink_left);
//        while (rx_flag == 1);

//        //uart0 send data
//        tx_flag = 1;
//        uart_write(QN_UART0, buffer, 10, led_blink_right);
//        while (tx_flag == 1);
    }
}

