/*******************************************************************************
* @file      	steep_motor.h
* @author    	Chen
* @version	 	1.0.1
* @date      	2018/03/01
* @brief     	步进电机控制		   
********************************************************************************
* @attention	1 Table = 4 Spacebar.  
*				默认24.5MHz时钟，12分频
* 			 	步进电机控制具有加减速，加减速自动控制，无需人为干预，
*				加减速仅需设置最低&最高频率及变频步长即可
********************************************************************************
* @license
*				All contributions by Chen:
*				Copyright (c) 2018 Chen.
*				All rights reserved.	 
*******************************************************************************/

#ifndef _steep_motor_H
#define _steep_motor_H_H

extern void Motor_Init (void);  				//步进电机初始化
extern void Motor_Stop(void);					//刹车
extern void Motor_Positive(unsigned long int Step);	//正向转动
extern void Motor_Reverse(unsigned long int Step);	//反向转动
extern unsigned char Find_Zero(void);			//寻零
extern unsigned char Init(void);				//衰减器第一次寻零成功后初始化
extern void Clear_STEP_Zero(void);				//相对零点步数清零(仅Hall.c文件调用)
extern unsigned long int Inquire_STEP(void);	//查询剩余步数
extern unsigned int Inquire_STEP_Zero(void);	//查询相对零点步数
extern unsigned char Motor_Boot(void);			//首次开机

extern void Timer2_Init(void);					//定时器2初始化
extern void Drive_Init(void);					//步进驱动初始化
extern void Motor(unsigned char Dir,unsigned long int Step);	//转电机

#endif

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///