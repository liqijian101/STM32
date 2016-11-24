#ifndef __USART2_DMA_H
#define __USART2_DMA_H
#include "stdio.h"	
#include "sys.h" 
struct _uart_buffer
{
	unsigned char  len;   //���ݳ���
	unsigned char  buf[64];  //�������ݻ�����
};
extern struct _uart_buffer uart_rx,uart_tx;

extern u8 USART2_TX_BUSY; //0������ 1:���ڷ���
void UART2_Init(void) ;
void USART2_DMA_Send_Once_Data(u8 *data,u16 size)  ;
u8 USART2_RX_Finish_IRQ(u8 *buf)  ;
#endif


