/*******************************************************************************
* @file      	usart.h
* @author    	Chen
* @version		2.1.1
* @date      	2018/03/07
* @brief     	����&��ʱ��3
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

extern void UART0_Init (unsigned long int BAUDRATE);//���ڳ�ʼ��
extern void Send(void); 							//����
extern signed char Receive(void);					//����
extern void Execution_Completed(void);				//ִ����ɣ��ش�����
extern void Send_Ratio(void);						//�ش�ÿȦ����
extern void Send_Error(unsigned char error);		//�ش��������
extern void Timer3_Init (void);						//��ʱ��3��ʼ��
extern void Timer3_Stop (void);						//ֹͣ��ʱ��3
extern void Loading_Finished(void);					//��ʼ�����

#endif

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///