/*******************************************************************************
* @file      	steep_motor.c
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

#include "c8051f310.h"
#include <math.h>
#include "steep_motor.h"
#include "flash.h"
#include "steep_motor.h"
#include "sys_Init.h"
#include "usart.h"

//------------------------------------------------------------------------------
sfr16 TMR2RL   = 0xca;		// Timer2 reload value 	Timer2����ֵ
sfr16 TMR2     = 0xcc; 		// Timer2 counter	Timer2������

sbit PULL = P0^0;			//����
sbit EN =  P0^1;			//ʹ��
sbit DIR = P0^2;			//����
sbit MS1 = P3^4;			//ϸ��
sbit MS2 = P3^3;			//ϸ��

sbit  Zero = P1^5;			//Zero2 ��������  0 Read

//------------------------------------------------------------------------------
#define Hertz_min 32							//����ٶ�32hz
#define Hertz_max 1000							//����ٶ�1600hz 1000
#define Steps_Low_speed 8						//���ٲ���  8
#define Frequency_conversion_Step_Default 200	//Ĭ�ϱ�Ƶ���� 200��
#define Frequency_conversion (Hertz_max - Hertz_min - 2 * Steps_Low_speed) / Frequency_conversion_Step			

#define Dir_Default 1				//Ĭ�ϳ�ʼ������
//#define Step_Default 3200			//Ĭ��һȦ3200��
#define Difference 32				//Ĭ�ϲ�ֵ 100 ��

//------------------------------------------------------------------------------
unsigned long int STEP = 0,STEP_Executed = 0;
signed int STEP_Zero = 100;			//�����㲽��
unsigned long int Frequency_conversion_Step = Frequency_conversion_Step_Default;	//ʵ�ʱ�Ƶ����

//--------------------------------- �����ʼ�� ---------------------------------
/**
* @name      	Motor_Init
* @brief     	���������ʼ��
* @param    	
* @retval	 
* @attention 			
*/
void Motor_Init (void)
{
	STEP = 1;
	Timer2_Init();
	Drive_Init();
}

/**
* @name      	Timer2_Init
* @brief     	��ʱ��2��ʼ��
* @param    	
* @retval	 
* @attention 			
*/
void Timer2_Init (void)
{
   CKCON &= ~0x60;                     	// Timer2 uses SYSCLK/12
   TMR2CN &= ~0x01;

   TMR2RL = 810; 						// Reload value to be used in Timer2
   TMR2 = TMR2RL;                      	// Init the Timer2 register

   TMR2CN = 0x04;                      	// Enable Timer2 in auto-reload mode
   ET2 = 1;                            	// Timer2 interrupt enabled   
	 TR2 = 0;
}

/**
* @name      	Drive_Init
* @brief     	�������������ʼ��
* @param    	
* @retval	 
* @attention 			
*/
void Drive_Init(void)
{
	EN = 1;				//ʧ��
	MS1 = 1;			//Ĭ�����ϸ�� 8ϸ��=16ϸ��
	MS2 = 1;
	DIR = Dir_Default;	//����Ĭ��
	EN = 0;
}

//----------------------------------- ��ʼ�� -----------------------------------
/**
* @name       	Init
* @brief     	˥������һ��Ѱ��ɹ����ʼ��
* @param     	null
* @retval	 	0:OK
*				1:�������س��չ���
*				2:�������س�������
*				3:΢������
* @attention 	�˹��̼̳���һ����Ƶ�ʣ�����Ĭ�ϲ��������ֹͣ		
*/
unsigned char Init(void)
{
	unsigned char count = 0;
//��һȦ������Ѱ��ä�죬3Ȧ֮��Ѱ��ʧ��������
	Delay500ms();				//��ʱ500ms
 	Motor_Positive(Read_Step_Default_Flash() * 3);
	while(!Zero){ 				//�����������ȴ�Խ��������
		if(!Inquire_STEP())		//�������س����쳣��һֱ������
		  return 1;
	}
	while(Zero){   				//�ȴ�����
		if(!Inquire_STEP())		//�������س����쳣����ⲻ����
			return 2;
	}
	STEP_Zero = 0;
//---------------------------------------
	STEP = Read_Step_Default_Flash() - Difference;	//�趨����
//---------------------------------------
	while(!Zero){ 				//�����������ȴ�Խ��������
		if(!Inquire_STEP())		//�������س����쳣
		   return 1;
	}
	while(Zero){   				//�ȴ�����
		if(!Inquire_STEP()){ 	//΢��
			count ++;
			Motor_Positive(1);
		}
		if(count >= 0.2 * Read_Step_Default_Flash())	//΢���쳣
			return 3;
	}
	Motor_Stop();
	STEP_Zero = 0;
	return 0;
}

/**
* @name       	Motor_Boot
* @brief      	�״ο���������ÿȦ����
* @param     	null
* @retval	 	0:OK
*				1:�������س��չ���
*				2:�������س�������
*				3:΢������
*				4:�ǲ���ֵ����
* @attention 			
*/
unsigned char Motor_Boot(void)
{
	unsigned char count = 0,i = 0;
	unsigned int Step_Temporary[4] = 0;		//�ݴ�ÿȦ����
	unsigned int Step_Default = 3200;		//Ĭ��һȦ3200��
//����ä��Ѱ�����
 	Motor_Positive(Step_Default * 6);
	while(!Zero){ 				//�����������ȴ�Խ��������
		if(!Inquire_STEP())		//�������س����쳣
		  return 1;
	}
	while(Zero){   				//�ȴ�����
		if(!Inquire_STEP())		//�������س����쳣
			return 2;
	}
//���ټǲ���Ȧ---------------------------------------	
	STEP_Zero = 0;
	STEP = Step_Default * 5;	//�趨����
	for(i = 0;i < 2;i++){
		while(!Zero){ 			//�����������ȴ�Խ��������
			if(!Inquire_STEP())	//�������س����쳣
				return 1;
		}
		while(Zero){   			//�ȴ�����
			if(!Inquire_STEP()) //�������س����쳣
				return 2;		//�������س�������
		}
		Step_Temporary[i] = STEP_Zero;	//��ȡ��һȦ����ÿȦ����
		STEP_Zero = 0;
	}
	if(abs(Step_Temporary[0] - Step_Temporary[1]) > Difference) return 4;//����ǲ���ֵ���󣬷���4
//����Ȧ���ٹ���---------------------------------------
	//�ж��Ƿ������ٽ׶�
	if(STEP > Frequency_conversion_Step_Default) 
		STEP = (Step_Temporary[0] + Step_Temporary[1]) / 2 - Difference;
	else {						//��������ٽ׶Σ�����ٽ������ٿ�ʼ��һ�ֲ���
		while(Inquire_STEP());
		Motor_Positive((Step_Temporary[0] + Step_Temporary[1]) / 2 - Difference / 2);
	}
	while(!Zero){ 				//�����������ȴ�Խ��������
		if(!Inquire_STEP())		//�������س����쳣
			return 1;
	}
	count = 0;
	while(Zero){   				//�ȴ�����
		if(!Inquire_STEP()){ 	//΢��
			count ++;
			Motor_Positive(1);
		}
		if(count >= 0.2 * (Step_Temporary[0] + Step_Temporary[1]) / 2)	//΢���쳣
			return 3;
	}
	Motor_Stop();
//���ټǲ���Ȧ---------------------------------------	
	for(i = 2;i < 4;i++){
		STEP_Zero = 0;		
		Motor_Positive((Step_Temporary[0] + Step_Temporary[1]) / 2 - Difference / 2);
		while(!Zero){ 			//�����������ȴ�Խ��������
			if(!Inquire_STEP())	//�������س����쳣
				return 1;
		}
		count = 0;
		while(Zero){   				//�ȴ�����
			if(!Inquire_STEP()){	//΢��
				count ++;
				Motor_Positive(1);
			}
			if(count >= 0.2 * (Step_Temporary[0] + Step_Temporary[1]) / 2)	//΢���쳣
				return 3;
		}
		Motor_Stop();
		Step_Temporary[i] = STEP_Zero;	//��ȡ��ȦÿȦ����
	}
	if(abs(Step_Temporary[2] - Step_Temporary[3]) > Difference) return 4;//������ټǲ���ֵ���󣬷���4
	if(abs(((Step_Temporary[0] + Step_Temporary[1]) / 2) - ((Step_Temporary[2] + Step_Temporary[3]) / 2)) > Difference) return 4;//������ټǲ���ֵ���󣬷���4
	Step_Default = (Step_Temporary[0] + Step_Temporary[1] + Step_Temporary[2] + Step_Temporary[3]) / 4 + 0.5;
	STEP_Zero = 0;
	Flash_Write(Step_Default);	
	return 0;
}

/**
* @name       	Find_Zero
* @brief     	Ѱ��
* @param     	null
* @retval	 	1:�������س��չ���
*				2:�������س�������
*				3:΢������
*				5:��γ���Ѱ��ʧ��
*				0:OK
* @attention 			
*/
unsigned char Find_Zero(void)
{
	unsigned int count = 0;
	static unsigned char Count_Retry = 0;
	
	Motor_Positive(Read_Step_Default_Flash() - STEP_Zero);
	while(!Zero){ 				//�����������ȴ�Խ��������
		if(!Inquire_STEP())		 
		   	return 1;			//�������س��չ���
	}
	while(Zero){   				//�ȴ�����
		if(!Inquire_STEP()){ 	//΢��
			count ++;
			Motor_Positive(1);
		}
		if(count >= 0.2 * Read_Step_Default_Flash())	
			return 3;			//΢���쳣
	}
	STEP_Zero= 0;
	//�ж��Ƿ������ٽ׶�
	if(STEP <= Steps_Low_speed) 
		Motor_Stop();			//����״ֱ̬��ֹͣ
	else {						//����״̬��˵����һ�����쳣�����ٺ�������Ѱ��
		if(STEP > Frequency_conversion_Step_Default) STEP = Frequency_conversion_Step_Default;//��ʼ����
		while(Inquire_STEP());	//�ȴ�ִ�����
		Find_Zero();			//����Ѱ��
		Count_Retry++;			//�ۼ����Դ���
		if(Count_Retry > 2) return 5;
	}
	Count_Retry = 0;
	Motor_Stop();
	return 0;
}

//------------------------------------ ��� -------------------------------------
/**
* @name       	Motor
* @brief     	�����������
* @param     	����&����
* @retval	 	null
* @attention 			
*/
void Motor(unsigned char Dir,unsigned long int Step)
{
	DIR = Dir;				//�趨����
	EN = 0;					//ʹ��
	if(!Step) return;
	//�жϲ����Ƿ���ɼӼ���ȫ����
	if(Step < 2 * Frequency_conversion_Step_Default) Frequency_conversion_Step = Step / 2;	//����ȫ����
	else Frequency_conversion_Step = Frequency_conversion_Step_Default;						//��ȫ����
	STEP = Step;			//�趨����
	STEP_Executed = 0;		//�����ִ�в���
	TMR2 = TMR2RL = -(2058672 / Hertz_min); 	//�趨��ʱ��2�Զ���װ��ֵ
	TR2 = 1;				//ʹ�ܶ�ʱ��2
	EA = 1;					//ʹ��ȫ���ж�
}

/**
* @name       	Motor_Positive
* @brief     	�����������
* @param     	����&����
* @retval	 	null
* @attention 			
*/
void Motor_Positive(unsigned long int Step)
{
	Motor(Dir_Default,Step);
}

/**
* @name       	Motor_Reverse
* @brief     	�����������
* @param     	����&����
* @retval	 	null
* @attention 			
*/
void Motor_Reverse(unsigned long int Step)
{
	Motor(!Dir_Default,Step);
}

/**
* @name       	Motor_Stop
* @brief     	�������ɲ��
* @param     	null
* @retval	 	null
* @attention 			
*/
void Motor_Stop(void)
{
	if(STEP <= Steps_Low_speed) {
		EN = 0;			//ʹ��
		TR2 = 0;		//ʹ�ܶ�ʱ��2
		STEP = 0;
	}
	else if(STEP >= Frequency_conversion_Step && STEP_Executed >= Frequency_conversion_Step )	//ȫ����ֹͣ
					STEP = Frequency_conversion_Step;	
	else if(STEP < Frequency_conversion_Step);			//������
	else if(STEP_Executed < Frequency_conversion_Step)	//������
					STEP = STEP_Executed;
	else{
//----------------------------------------	
		EN = 0;			//ʹ��
		TR2 = 0;		//ʹ�ܶ�ʱ��2
		STEP = 0;
	}
}

//------------------------------------ ��ѯ -------------------------------------
/**
* @name      	Inquire_STEP
* @brief     	��ѯʣ�ಽ��
* @param    	
* @retval	 	ʣ�ಽ��
* @attention 			
*/
unsigned long int Inquire_STEP(void)
{
	return STEP;
}

/**
* @name      	Inquire_STEP
* @brief     	��ѯ�����㲽��
* @param    	
* @retval	 	�����㲽��
* @attention 			
*/
unsigned int Inquire_STEP_Zero(void)
{
	while(STEP_Zero < 0)
		STEP_Zero += Read_Step_Default_Flash();
	if(STEP_Zero > Read_Step_Default_Flash()) 
		return STEP_Zero % Read_Step_Default_Flash();
	return STEP_Zero;
}

/**
* @name      	Clear_STEP_Zero
* @brief     	�����㲽������
* @param    	
* @retval	 
* @attention 			
*/
void Clear_STEP_Zero(void)
{
	STEP_Zero = 0;
}

//------------------------------------ �ж� -------------------------------------
/**
* @name      	Timer2_ISR
* @brief     	��ʱ��2�ж�
* @param    
* @retval	 
* @attention 			
*/
void Timer2_ISR(void) interrupt 5
{
	PULL = 0;
	STEP--;
	STEP_Executed++;	
	if(!STEP) TR2 = 0;  		//�������꣬�رն�ʱ��2
	
	if(DIR == Dir_Default) STEP_Zero++;		//�����㲽����Ĭ�Ϸ���ͬ�� +1
	else STEP_Zero--;						//�����㲽����Ĭ�Ϸ����� -1
	
	if(STEP <= Steps_Low_speed || STEP_Executed <= Steps_Low_speed) //����������
		TMR2 = TMR2RL = -(2058672 / Hertz_min);
	if(STEP < Frequency_conversion_Step) 				//����
		TMR2 = TMR2RL = -(2058672 / (Hertz_min + STEP * Frequency_conversion)); // Reload value to be used in Timer2
	else if(STEP_Executed < Frequency_conversion_Step)	//����
					TMR2 = TMR2RL = -(2058672 / (Hertz_min + STEP_Executed * Frequency_conversion)); // Reload value to be used in Timer2
	
	PULL = 1;
	TF2H = TF2L = 0;			//���������
}

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///