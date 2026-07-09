#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	
#include "usart.h"	

#define RX_BUF_SZIE     1024




extern uint8_t usart1_rxbuf[RX_BUF_SZIE];

extern void uart_init(uint32_t bound);

#endif
