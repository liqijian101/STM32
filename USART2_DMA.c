#include "sys.h"
#include "USART2_DMA.h"
#include <string.h>
struct _uart_buffer uart_rx,uart_tx;
	  
/*******************************************************************************
1、后台数据-》USARTx-》其它设备，其它设备数据-》USARTx-》后台，这两个数据过程也可能同时进行。
*********************************************************************************/

#define USART2_DATA_LEN  64  //接收和发送数据的最大长度
#define USART2_BAUD 115200    //串口1波特率  

u8 USART2_SEND_DATA[USART2_DATA_LEN];  
u8 USART2_RECEIVE_DATA[USART2_DATA_LEN];
u8 USART2_TX_BUSY=0; //0：空闲 1:正在发送

void UART2_Init(void)  
{
 //定义中断结构体  
    NVIC_InitTypeDef NVIC_InitStructure ;  
    //定义IO初始化结构体  
    GPIO_InitTypeDef GPIO_InitStructure;  
    //定义串口结构体    
    USART_InitTypeDef USART_InitStructure;  
    //定义DMA结构体  
    DMA_InitTypeDef DMA_InitStructure;  

//串口IO设置
    //第2步：打开GPIO和USART2部件的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);	
	//第2步：将USART2 Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 第3步：将USART2 Rx的GPIO配置为浮空输入模式
	//	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
	//	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


//基本串口参数设置
 	//打开串口对应的外设时钟    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  

	USART_InitStructure.USART_BaudRate = USART2_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//开校验时,校验位算在数据位中,因此9个数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
	// CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
  	//如下语句解决第1个字节无法正确发送出去的问题 
  	USART_ClearFlag(USART2, USART_FLAG_TC);     //清发送外城标志，Transmission Complete flag 
//串口2中断配置
  	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
    USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
    USART_ITConfig(USART1,USART_IT_TXE,DISABLE); 
	USART_ITConfig(USART2, USART_IT_IDLE , ENABLE);//关闭空闲中断 

  	// Enable USART2 DMA Rx Tx request 
  	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); 
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);



//串口发DMA配置    
    //启动DMA时钟 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DMA1
	//DMA通道配置	DMA1 Channel5-->USART2 Tx
  	DMA_DeInit(DMA1_Channel7);
	   
  	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);	 	//外设地址
  	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_SEND_DATA;		//内存地址																	
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						//DMA传输方向												
  	DMA_InitStructure.DMA_BufferSize = USART2_DATA_LEN;						//设置DMA在传输时缓冲区的长度
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置DMA的外设递增模式，一个外设 	
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置DMA的内存递增模式 	
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据字长	 
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//内存数据字长	 
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//设置DMA的传输模式
  	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//设置DMA的优先级别
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  	DMA_Init(DMA1_Channel7, &DMA_InitStructure);							//配置DMA2的通道
  
  	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);						    //使能传输完成中断 

    DMA_Cmd(DMA1_Channel7, DISABLE);
	//DMA发送中断设置 	Enable DMA Channel7 Interrupt 
 	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
   
   
//串口收DMA配置    
    //启动DMA时钟 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DMA1
	//DMA通道配置 DMA1 Channel6 --> USART2 Rx 
  	DMA_DeInit(DMA1_Channel6);  
  	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&USART2->DR);		//外设地址
  	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2_RECEIVE_DATA;	//内存地址
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						//DMA传输方向
  	DMA_InitStructure.DMA_BufferSize =USART2_DATA_LEN;						//设置DMA在传输时缓冲区的长度
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置DMA的外设递增模式，一个外设  
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置DMA的内存递增模式 
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据字长
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		    //内存数据字长
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//设置DMA的传输模式
  	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;					//设置DMA的优先级别
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  	DMA_Init(DMA1_Channel6, &DMA_InitStructure);							//配置DMA2的通道   
  
//  	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);							//使能传输完成中断
//  
//	//DMA接收中断设置  Enable DMA Channel6 Interrupt 
//  	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  	NVIC_Init(&NVIC_InitStructure);

  	DMA_Cmd(DMA1_Channel6, ENABLE);

}
/********************************************************************* 
*                           接口函数:串口通过DMA向外发送数据 
*参数:data:发送数据存放地址 
*     size:发送数据字节数 
**********************************************************************/  
void USART2_DMA_Send_Once_Data(u8 *data,u16 size)  
{
    //等待空闲  
    while (USART2_TX_BUSY);  
    USART2_TX_BUSY = 1;  
    //复制数据  
    memcpy(USART2_SEND_DATA,data,size);  
    //USART用DMA传输替代查询方式发送，克服被高优先级中断而产生丢帧现象。
    DMA_Cmd(DMA1_Channel7, DISABLE); //改变datasize前先要禁止通道工作
    DMA1_Channel7->CNDTR=size; //DMA1,传输数据量
    DMA_Cmd(DMA1_Channel7, ENABLE);	
}

  
/********************************************************************* 
*                           串口2处理接收完成中断 
*参数:buf:接收的数据 
*     len:接收的数据长度 
*返回:0:未产生,其他:已经产生,此值为接收的数据长度 
**********************************************************************/  
  
u8 USART2_RX_Finish_IRQ(u8 *buf)  
{     
    u16 len = 0;  
 
    USART2->SR;  
    USART2->DR; //清USART_IT_IDLE标志  
    //关闭DMA  
    DMA_Cmd(DMA1_Channel6, DISABLE);//关闭DMA,防止处理其间有数据
    //清除标志位
	  DMA_ClearITPendingBit(DMA1_IT_TC6);
          
    //获得接收帧帧长 
	len=USART2_DATA_LEN-DMA_GetCurrDataCounter(DMA1_Channel6); 
    memcpy(buf,USART2_RECEIVE_DATA,len);  
          
    //设置传输数据长度  	
    DMA_SetCurrDataCounter(DMA1_Channel6,USART2_DATA_LEN);  
    //打开DMA  
    DMA_Cmd(DMA1_Channel6, ENABLE);//处理完,重开DMA 
    return len;   
}  

/********************************************************************* 
*                         串口中断处理函数 
**********************************************************************/  

void USART2_IRQHandler(void) 		 //单独使用本文件是用他，现在把他放到stm32f4xx_it.c中    
{  
      
    //发送完成中断处理  
    if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)  
    {  
        //关闭发送完成中断  
        USART_ITConfig(USART2,USART_IT_TC,DISABLE); 
        //发送完成  
        USART2_TX_BUSY = 0;  
    }   
      
    //接收完成中断处理 
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
    	uart_rx.len = USART2_RX_Finish_IRQ(uart_rx.buf);
 
		/*  if (uart_rx.len != 0)  
    		{  
    
    		} */
	} 
  
}

//USART2使用DMA发数据中断服务程序
void DMA1_Channel7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC7) != RESET)   
    {  
        //清除标志位  
        DMA_ClearITPendingBit(DMA1_IT_TC7);  
        //关闭DMA  
        DMA_Cmd(DMA1_Channel7, DISABLE); 
        //打开发送完成中断,发送最后两个字节  
        USART_ITConfig(USART2,USART_IT_TC,ENABLE);  
    }  
}

//void DMA1_Channel6_IRQHandler(void)
//{
//	uart_rx.len = USART2_RX_Finish_IRQ(uart_rx.buf);
//	
//}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/






