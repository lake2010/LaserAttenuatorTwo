/*******************************************************************************
* @file      	Flash.c
* @author    	Chen
* @version  	2.0.1
* @date      	2018/03/01
* @brief     	flash��ȡ  
********************************************************************************
* @Email     	Chen.0810@outlook.com
* @Github    	https://github.com/Chen-0810
********************************************************************************
* @attention	1 Table = 4 Spacebar.  
*            	FLASH��ģ16384�ֽڣ�����0x3E00~0x3FFF����
*            	������ʹ����0x3A00���9�ֽڣ���������0x3C00~0x3E00
********************************************************************************
* @license
*            	All contributions by Chen:
*            	Copyright (c) 2018 Chen.
*            	All rights reserved.	 
*******************************************************************************/

#include "c8051f310.h"
#include "flash.h"
#include "steep_motor.h"

//------------------------------------------------------------------------------
#define Length_Flash  9  		//flash����֡����
#define Flash_Start   0x3A00	//flash��ʼ�洢λ��
#define Frame_Start_0 0x43		//֡ͷ0
#define Frame_Start_1 0x68		//֡ͷ1
#define Frame_End_0   0x65		//֡β0
#define Frame_End_1   0x6E		//֡β1

//------------------------------------------------------------------------------
unsigned int Step_Zero_Flash = 0;  			//flash�������Ĳ���
unsigned int Step_Default_Flash = 0;		//Ĭ��һȦ3200��

//----------------------------------- ��ʼ�� -----------------------------------
/**
* @name			Flash_Init
* @brief		Flash��ʼ��
* @param     
* @retval		0:flash����
*				1:�״ο���
*				2:OK
* @attention 			
*/
unsigned char Flash_Init(void)
{	
	signed int Cache = Flash_Read();
	if(Cache < -1)		//flash����
		return 0;
	if(Cache == -1){	//�״ο���
		Step_Zero_Flash = 0;
		return 1;
	}
	else Step_Zero_Flash = Cache;
	return 2;
}

//----------------------------- ����Flash -----------------------------
/**
* @name			Read_Step_Flash
* @brief		��ȡflash�в���
* @param     
* @retval	 		
* @attention 			
*/
unsigned int Read_Step_Flash(void)
{	
	return Step_Zero_Flash;
}

/**
* @name			Read_Step_Default_Flash
* @brief		��ȡflash��ÿȦ����
* @param     
* @retval	 		
* @attention 			
*/
unsigned int Read_Step_Default_Flash(void)
{	
	return Step_Default_Flash;
}

/**
* @name			Flash_Write
* @brief		д��Flash
* @param     
* @retval		0:OK
*				1:д��ʧ��
* @attention 			
*/
unsigned char Flash_Write(unsigned int Step_Default)
{
	unsigned char flash[Length_Flash] = 
		{Frame_Start_0,Frame_Start_1,0x00,0x00,0x00,0x00,0x00,Frame_End_0,Frame_End_1};
									//Ԥд��flash����ֵ 
	unsigned char Position_Flash;	//flash��ǰλ��
  	unsigned char xdata *Flash_write;//дָ�룬
	unsigned char code *Flash_read;	//��ָ�룬
  	unsigned char EA_Save;  		//�ݴ��жϱ�־
	unsigned char CheckSum = 0;		//У���
	unsigned int 	Step = 0;		//�ݴ浱ǰλ�ò���
		
	if(Step_Default == 0)	{		//д�븴λ�㣬���漰д��ÿȦ������Ӧ����flash�д洢��ÿȦ����
		Flash_read = (unsigned char code *) Flash_Start;	
		flash[2] = *(Flash_read + 2);
		flash[3] = *(Flash_read + 3);
		
		Step = Inquire_STEP_Zero();	//��ȡ��ǰ����
		flash[4] = (Step & 0xFF00) >> 8;		
		flash[5] = (Step & 0x00FF);		
	}
	else {							//��һ���ϵ磬��flashд��ÿȦ����
		flash[2] = Step_Default >> 8;
		flash[3] = Step_Default;
		Step_Default_Flash = Step_Default;
		flash[4] = 0;		
		flash[5] = 0;	
	}
	
	//����У���	
	for(Position_Flash = 2;Position_Flash < Length_Flash - 3;Position_Flash++){
		CheckSum += flash[Position_Flash];
	}
	flash[Position_Flash] = CheckSum;
	
	RSTSRC |= 0x02;		//ʹ��Ƭ��VDD���                                    	
	PCA0MD &= ~0x40;	//���ÿ��Ź���ʱ�� 										  
	EA_Save = EA;		//�����ж� 
  	EA = 0;				//�����ж�(Ԥ����)
    
	//��������	
	Flash_write = (unsigned char xdata *) Flash_Start;//ָ��ָ��Ŀ��λ�ã�׼����������
	PSCTL = 0x03;                         	//׼������flash���� 512�ֽ�
	FLKEY = 0xA5;                         	//д���һ��FLASH�ؼ���0xA5
	FLKEY = 0xF1;                         	//д��ڶ���FLASH�ؼ���0xF1
	*Flash_write = 0x00;                  	//д������ֵ�Բ�����ҳ��
	PSCTL = 0;                            	//������PSWEλ

	//д��	
	EA= 0;					//��ֹ�ж�
 	Flash_write = Flash_Start;            	//ָ��ָ��Ŀ��λ�ã�׼��д������
	PSCTL = 0x01;                         	//��λPSWE,дFlash
	for(Position_Flash = 0;Position_Flash < Length_Flash;Position_Flash++) {    //��ʼд��Flash
		FLKEY = 0xA5;                   	//д���һ��FLASH�ؼ���0xA5
		FLKEY = 0xF1;                   	//д��ڶ���FLASH�ؼ���0xF1
  	*Flash_write = flash[Position_Flash];   //д������
  	Flash_write++;
	}
 	PSCTL = 0x00;                         	//
 	EA = EA_Save;                         	//�ָ�ȫ���ж�
	
	if(!Step_Default && Flash_Read() != Step) return 1;		//д��ʧ��
	return 0;
}

/**
* @name			Flash_Read
* @brief		��ȡFlash
* @param     
* @retval		-2:֡ͷ��֡β��У������
*				-1:�״ο���
*				xx:�궨����
* @attention 			
*/
signed int Flash_Read(void)
{
	unsigned char flash[Length_Flash]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//Ԥд��flash����ֵ
	unsigned char code *Flash_read; 
	unsigned char Position_Flash;	//flash��ǰλ��
	unsigned char CheckSum = 0;		//У���
	unsigned char Flag_Boot = 0;	//�״ο�����־
	
	//����flash
	Flash_read = (unsigned char code *) Flash_Start;
	for(Position_Flash = 0;Position_Flash < Length_Flash;Position_Flash++){	
		flash[Position_Flash] = *(Flash_read + Position_Flash);
		if(flash[Position_Flash] == 0xFF) Flag_Boot++;
	}	
	
	if(Flag_Boot == Length_Flash) return -1;	//�״ο���
		
	//����У���
	for(Position_Flash = 2;Position_Flash < Length_Flash - 3;Position_Flash++){
		CheckSum += flash[Position_Flash];
	}
	if((flash[0] != Frame_Start_0) || (flash[1] != Frame_Start_1) 
		|| (flash[Length_Flash - 2] != Frame_End_0) || (flash[Length_Flash - 1] != Frame_End_1)
		|| (CheckSum != flash[Length_Flash - 3])) 
		return -2;				//֡ͷ��֡β��У�������˳�
	
	Step_Default_Flash = ((flash[2] & 0x00FF) << 8) | flash[3];		//flash�д洢��ÿȦ����
	
	return ((flash[4] & 0x00FF) << 8) | flash[5];	//���ر궨����
}

/**
* @name			Flash_Reset
* @brief		�ָ��������ã�����Flash����
* @param     
* @retval		1:OK
* @attention 			
*/
unsigned char Flash_Reset(void)
{
	unsigned char xdata *Flash_write;	//ָ�룬
  	unsigned char EA_Save;  			//�ݴ��жϱ�־
	
	RSTSRC |= 0x02;                   	//ʹ��Ƭ��VDD���                                    	
	PCA0MD &= ~0x40;             		//���ÿ��Ź���ʱ�� 										  
	EA_Save = EA;						//�����ж� 
  	EA = 0;                             //�����ж�(Ԥ����)
    
	//��������	
	Flash_write = (unsigned char xdata *) Flash_Start;//ָ��ָ��Ŀ��λ�ã�׼����������
	PSCTL = 0x03;                        //׼������flash���� 512�ֽ�
	FLKEY = 0xA5;                        //д���һ��FLASH�ؼ���0xA5
	FLKEY = 0xF1;                        //д��ڶ���FLASH�ؼ���0xF1
	*Flash_write = 0;                    //д������ֵ�Բ�����ҳ��
	PSCTL = 0;                           //������PSWEλ

 	EA = EA_Save;                        //�ָ�ȫ���ж�
	return 1;
}

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///