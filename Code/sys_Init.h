/*******************************************************************************
* @file      	sys_Init.h
* @author    	Chen
* @version   	2.0.1
* @date      	2018/03/01
* @brief     	ϵͳ��ʼ��		   
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

#ifndef _sys_Init_H
#define _sys_Init_H_H

extern void sys_Init (void); 	//ϵͳ��ʼ��
extern void Restart(void);		//����
extern void Delay500ms(void);	//��ʱ500ms
extern void Delay10ms(void);	//��ʱ10ms

extern void SYSCLK_Init (void);	//ϵͳʱ�ӳ�ʼ��
extern void PORT_Init (void);	//I/O�ڳ�ʼ��
extern void Watchdog(void);		//��ʼ�����Ź�

#endif

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///