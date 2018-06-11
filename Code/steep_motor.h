/*******************************************************************************
* @file      	steep_motor.h
* @author    	Chen
* @version	 	1.0.1
* @date      	2018/03/01
* @brief     	�����������		   
********************************************************************************
* @attention	1 Table = 4 Spacebar.  
*				Ĭ��24.5MHzʱ�ӣ�12��Ƶ
* 			 	����������ƾ��мӼ��٣��Ӽ����Զ����ƣ�������Ϊ��Ԥ��
*				�Ӽ��ٽ����������&���Ƶ�ʼ���Ƶ��������
********************************************************************************
* @license
*				All contributions by Chen:
*				Copyright (c) 2018 Chen.
*				All rights reserved.	 
*******************************************************************************/

#ifndef _steep_motor_H
#define _steep_motor_H_H

extern void Motor_Init (void);  				//���������ʼ��
extern void Motor_Stop(void);					//ɲ��
extern void Motor_Positive(unsigned long int Step);	//����ת��
extern void Motor_Reverse(unsigned long int Step);	//����ת��
extern unsigned char Find_Zero(void);			//Ѱ��
extern unsigned char Init(void);				//˥������һ��Ѱ��ɹ����ʼ��
extern void Clear_STEP_Zero(void);				//�����㲽������(��Hall.c�ļ�����)
extern unsigned long int Inquire_STEP(void);	//��ѯʣ�ಽ��
extern unsigned int Inquire_STEP_Zero(void);	//��ѯ�����㲽��
extern unsigned char Motor_Boot(void);			//�״ο���

extern void Timer2_Init(void);					//��ʱ��2��ʼ��
extern void Drive_Init(void);					//����������ʼ��
extern void Motor(unsigned char Dir,unsigned long int Step);	//ת���

#endif

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///