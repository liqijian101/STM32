#include "sys.h"
#include "USART2_DMA.h"
#include <string.h>
struct _uart_buffer uart_rx,uart_tx;
	  
/*******************************************************************************
1����̨����-��USARTx-�������豸�������豸����-��USARTx-����̨�����������ݹ���Ҳ����ͬʱ���С�
*********************************************************************************/

#define USART2_DATA_LEN  64  //���պͷ������ݵ���󳤶�
#define USART2_BAUD 115200    //����1������  

u8 USART2_SEND_DATA[USART2_DATA_LEN];  
u8 USART2_RECEIVE_DATA[USART2_DATA_LEN];
u8 USART2_TX_BUSY=0; //0������ 1:���ڷ���

void UART2_Init(void)  
{
 //�����жϽṹ��  
    NVIC_InitTypeDef NVIC_InitStructure ;  
    //����IO��ʼ���ṹ��  
    GPIO_InitTypeDef GPIO_InitStructure;  
    //���崮�ڽṹ��    
    USART_InitTypeDef USART_InitStructure;  
    //����DMA�ṹ��  
    DMA_InitTypeDef DMA_InitStructure;  

//����IO����
    //��2������GPIO��USART2������ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);	
	//��2������USART2 Tx��GPIO����Ϊ���츴��ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ��3������USART2 Rx��GPIO����Ϊ��������ģʽ
	//	����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
	//	���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


//�������ڲ�������
 	//�򿪴��ڶ�Ӧ������ʱ��    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  

	USART_InitStructure.USART_BaudRate = USART2_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��У��ʱ,У��λ��������λ��,���9������λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
	// CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
  	//�����������1���ֽ��޷���ȷ���ͳ�ȥ������ 
  	USART_ClearFlag(USART2, USART_FLAG_TC);     //�巢����Ǳ�־��Transmission Complete flag 
//����2�ж�����
  	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
    USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
    USART_ITConfig(USART1,USART_IT_TXE,DISABLE); 
	USART_ITConfig(USART2, USART_IT_IDLE , ENABLE);//�رտ����ж� 

  	// Enable USART2 DMA Rx Tx request 
  	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); 
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);



//���ڷ�DMA����    
    //����DMAʱ�� 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DMA1
	//DMAͨ������	DMA1 Channel5-->USART2 Tx
  	DMA_DeInit(DMA1_Channel7);
	   
  	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);	 	//�����ַ
  	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_SEND_DATA;		//�ڴ��ַ																	
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						//DMA���䷽��												
  	DMA_InitStructure.DMA_BufferSize = USART2_DATA_LEN;						//����DMA�ڴ���ʱ�������ĳ���
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//����DMA���������ģʽ��һ������ 	
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//����DMA���ڴ����ģʽ 	
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //���������ֳ�	 
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�ڴ������ֳ�	 
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//����DMA�Ĵ���ģʽ
  	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//����DMA�����ȼ���
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  	DMA_Init(DMA1_Channel7, &DMA_InitStructure);							//����DMA2��ͨ��
  
  	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);						    //ʹ�ܴ�������ж� 

    DMA_Cmd(DMA1_Channel7, DISABLE);
	//DMA�����ж����� 	Enable DMA Channel7 Interrupt 
 	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
   
   
//������DMA����    
    //����DMAʱ�� 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DMA1
	//DMAͨ������ DMA1 Channel6 --> USART2 Rx 
  	DMA_DeInit(DMA1_Channel6);  
  	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&USART2->DR);		//�����ַ
  	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_RECEIVE_DATA;	//�ڴ��ַ
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						//DMA���䷽��
  	DMA_InitStructure.DMA_BufferSize =USART2_DATA_LEN;						//����DMA�ڴ���ʱ�������ĳ���
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//����DMA���������ģʽ��һ������  
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//����DMA���ڴ����ģʽ 
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//���������ֳ�
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		    //�ڴ������ֳ�
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//����DMA�Ĵ���ģʽ
  	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;					//����DMA�����ȼ���
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  	DMA_Init(DMA1_Channel6, &DMA_InitStructure);							//����DMA2��ͨ��   
  
//  	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);							//ʹ�ܴ�������ж�
//  
//	//DMA�����ж�����  Enable DMA Channel6 Interrupt 
//  	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  	NVIC_Init(&NVIC_InitStructure);

  	DMA_Cmd(DMA1_Channel6, ENABLE);

}
/********************************************************************* 
*                           �ӿں���:����ͨ��DMA���ⷢ������ 
*����:data:�������ݴ�ŵ�ַ 
*     size:���������ֽ��� 
**********************************************************************/  
void USART2_DMA_Send_Once_Data(u8 *data,u16 size)  
{
    //�ȴ�����  
    while (USART2_TX_BUSY);  
    USART2_TX_BUSY = 1;  
    //��������  
    memcpy(USART2_SEND_DATA,data,size);  
    //USART��DMA���������ѯ��ʽ���ͣ��˷��������ȼ��ж϶�������֡����
    DMA_Cmd(DMA1_Channel7, DISABLE); //�ı�datasizeǰ��Ҫ��ֹͨ������
    DMA1_Channel7->CNDTR=size; //DMA1,����������
    DMA_Cmd(DMA1_Channel7, ENABLE);	
}

  
/********************************************************************* 
*                           ����2�����������ж� 
*����:buf:���յ����� 
*     len:���յ����ݳ��� 
*����:0:δ����,����:�Ѿ�����,��ֵΪ���յ����ݳ��� 
**********************************************************************/  
  
u8 USART2_RX_Finish_IRQ(u8 *buf)  
{     
    u16 len = 0;  
 
    USART2->SR;  
    USART2->DR; //��USART_IT_IDLE��־  
    //�ر�DMA  
    DMA_Cmd(DMA1_Channel6, DISABLE);//�ر�DMA,��ֹ�������������
    //�����־λ
	  DMA_ClearITPendingBit(DMA1_IT_TC6);
          
    //��ý���֡֡�� 
	len=USART2_DATA_LEN-DMA_GetCurrDataCounter(DMA1_Channel6); 
    memcpy(buf,USART2_RECEIVE_DATA,len);  
          
    //���ô������ݳ���  	
    DMA_SetCurrDataCounter(DMA1_Channel6,USART2_DATA_LEN);  
    //��DMA  
    DMA_Cmd(DMA1_Channel6, ENABLE);//������,�ؿ�DMA 
    return len;   
}  

/********************************************************************* 
*                         �����жϴ����� 
**********************************************************************/  

void USART2_IRQHandler(void) 		 //����ʹ�ñ��ļ������������ڰ����ŵ�stm32f4xx_it.c��    
{  
      
    //��������жϴ���  
    if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)  
    {  
        //�رշ�������ж�  
        USART_ITConfig(USART2,USART_IT_TC,DISABLE); 
        //�������  
        USART2_TX_BUSY = 0;  
    }   
      
    //��������жϴ��� 
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
    	uart_rx.len = USART2_RX_Finish_IRQ(uart_rx.buf);
 
		/*  if (uart_rx.len != 0)  
    		{  
    
    		} */
	} 
  
}

//USART2ʹ��DMA�������жϷ������
void DMA1_Channel7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC7) != RESET)   
    {  
        //�����־λ  
        DMA_ClearITPendingBit(DMA1_IT_TC7);  
        //�ر�DMA  
        DMA_Cmd(DMA1_Channel7, DISABLE); 
        //�򿪷�������ж�,������������ֽ�  
        USART_ITConfig(USART2,USART_IT_TC,ENABLE);  
    }  
}

//void DMA1_Channel6_IRQHandler(void)
//{
//	uart_rx.len = USART2_RX_Finish_IRQ(uart_rx.buf);
//	
//}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/






