/*******************************************************************************
* @file      	sys_Init.h
* @author    	Chen
* @version   	2.0.1
* @date      	2018/03/01
* @brief     	系统初始化		   
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

extern void sys_Init (void); 	//系统初始化
extern void Restart(void);		//重启
extern void Delay500ms(void);	//延时500ms
extern void Delay10ms(void);	//延时10ms

extern void SYSCLK_Init (void);	//系统时钟初始化
extern void PORT_Init (void);	//I/O口初始化
extern void Watchdog(void);		//初始化看门狗

#endif

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///