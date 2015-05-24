#include <stm32f10x_lib.h>
#include "stdio.h"	 
	  	
extern u8 USART_RX_BUF[256];    //RXT buffer
extern u8 USART_RX_STA;         //compleation flag	
extern u8 USART_RX_CNT;         //RXT count

void uart_init(u32 pclk2,u32 bound);

















