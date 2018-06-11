/*******************************************************************************
* @file      	usart.h
* @author    	Chen
* @version		2.1.1
* @date      	2018/03/07
* @brief     	串口&定时器3
********************************************************************************
* @Email     	Chen.0810@outlook.com
* @Github  		https://github.com/Chen-0810
********************************************************************************
* @attention	1 Table = 4 Spacebar. 		 
********************************************************************************
* @license
*            	All contributions by Chen:
*            	Copyright (c) 2018 Chen.
*            	All rights reserved.	 
*******************************************************************************/

#ifndef _usart_H
#define _usart_H_H

extern void UART0_Init (unsigned long int BAUDRATE);//串口初始化
extern void Send(void); 							//发送
extern signed char Receive(void);					//接收
extern void Execution_Completed(void);				//执行完成，回传数据
extern void Send_Ratio(void);						//回传每圈步数
extern void Send_Error(unsigned char error);		//回传错误代号
extern void Timer3_Init (void);						//定时器3初始化
extern void Timer3_Stop (void);						//停止定时器3
extern void Loading_Finished(void);					//初始化完成

#endif

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///