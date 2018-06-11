/*******************************************************************************
* @file      	main.c
* @author    	Chen
* @version   	2.1.1
* @date      	2018/04/28
* @brief     	����˥����		   
********************************************************************************
* @Email     	Chen.0810@outlook.com
* @Github    	https://github.com/Chen-0810
********************************************************************************
* @attention	1 Table = 4 Spacebar.
*            	�ϵ缴��λ ��
********************************************************************************
* @license
*            	All contributions by Chen:
*            	Copyright (c) 2018 Chen.
*            	All rights reserved.	 
*******************************************************************************/

#include "c8051f310.h"
#include "intrins.h"
#include "flash.h"
#include "steep_motor.h"
#include "sys_Init.h"
#include "usart.h"

//------------------------------------------------------------------------------
sfr16 DP       = 0x82;                 	// data pointer		����ָ��
sfr16 PCA0CP0  = 0xfb;                 	// PCA0 Module 0 Capture/Compare		
sfr16 PCA0CP1  = 0xe9;                 	// PCA0 Module 1 Capture/Compare		
sfr16 PCA0CP2  = 0xeb;                 	// PCA0 Module 2 Capture/Compare		
sfr16 PCA0CP3  = 0xed;                 	// PCA0 Module 3 Capture/Compare		
sfr16 PCA0CP4  = 0xfd;                 	// PCA0 Module 4 Capture/Compare			
sfr16 PCA0     = 0xf9;                 	// PCA0 counter		PCA0������

//------------------------------------------------------------------------------
#define Failed_passbacks	5	//ʧ�ܻش�����

//----------------------------------- ������ -----------------------------------
/**
* @name      	main
* @brief     	������
* @param    
* @retval	 
* @attention 			
*/
void main(void){
	unsigned char Return = 0;
	unsigned char Count = 0;
//-------------------- ��ʼ�� ----------------------------	
	sys_Init();			//ϵͳ��ʼ��
	Motor_Init();		//�����ʼ��
	UART0_Init(115200);	//��ʼ�����ڣ�������115200bps
 	EA = 1;				//ʹ��ȫ���ж�
	
	Timer3_Init();		//����ʵʱ�ش�ָ��
	
	if(Flash_Init() == 1){			//�״ο���
		Return = Motor_Boot();		//����ÿȦ����
		if(Return) {				//����ʧ�ܣ����ض�Ӧ�������x�β�����
			for(Count = 0;Count < Failed_passbacks;Count++){		
				Send_Error(Return);	//���ش������	
				Delay500ms();Delay500ms();
			}
			Restart();			//����
		}
		else Send_Ratio();		//�����ɹ����ش�ÿȦ����
	}
	else if(!Flash_Init()) {	//flash���󣬻ش��������
		while(1){				//�ش�����ָ���ҵȴ���λ��ָ�����/���ã�
			Send_Error(0x0D);	//flash���󣬻ش��������	
			Delay500ms();Delay500ms();
			}
		}
		else {					//��ȡflash�ɹ�	
			Return = Init();	//�ϵ�������ʼ��
			if(Return){ 		//��ʼ��ʧ�ܣ����ض�Ӧ�������x�β�����
				for(Count = 0;Count < Failed_passbacks;Count++){		
					Send_Error(Return);			//���ش������
					Delay500ms();Delay500ms();
				}
				Restart();		//����
			}
			Motor_Positive(Read_Step_Flash());	//��λ
		}		
			
	Delay10ms();	
	while(Inquire_STEP());		//�ȴ���λָ��ִ�����		
	Timer3_Stop();				//�ر�ʵʱ�ش�ָ��
	Loading_Finished();			//��ʼ�����
	Send();						//�ش���ʼ�����ָ��

//------------------- �������� ----------------------------
	while(1){
		if(Receive() == 1)		//���յ���ȷ����
			Return = 1;
		if(!Inquire_STEP() && (Return == 1)){	//���յ���ȷ������ִ�����
			Motor_Stop();				//ɲ��
			Return  = 0;
			Execution_Completed();		//�ش��ڶ�֡���ݣ�����ִ�����
		}
	}
}

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///
