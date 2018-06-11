/*******************************************************************************
* @file      	usart.c
* @author    	Chen
* @version		2.1.2
* @date      	2018/06/06
* @brief     	����&��ʱ��3
********************************************************************************
* @Email     	Chen.0810@outlook.com
* @Github  		https://github.com/Chen-0810
********************************************************************************
* @attention	1 Table = 4 Spacebar.		 
********************************************************************************
* @license
*            	All contributions by Chen.
*            	Copyright (c) 2018 Chen.
*            	All rights reserved.	 
********************************************************************************
* @Update		���ӣ���������֡���󣬻ش� 0xE6 �������		 
*******************************************************************************/

#include "c8051f310.h"
#include "usart.h"
#include "flash.h"
#include "steep_motor.h"
#include "sys_Init.h"
#include "usart.h"

#define SYSCLK          24500000       	// SYSCLK frequency in Hz	Ƶ��

//------------------------------------------------------------------------------
sfr16 TMR3RL   = 0x92;                 // Timer3 reload value   Timer3����ֵ
sfr16 TMR3     = 0x94;                 // Timer3 counter	Timer3������


//------------------------------------------------------------------------------
sbit Re = P3^1;		//����ʹ�� �͵�ƽ��Ч
sbit De = P3^2;		//����ʹ�� �ߵ�ƽ��Ч

//------------------------------------------------------------------------------
#define Length_Send 6							//���ͻ���������
#define Length_Receive 6						//���ջ���������
unsigned char BUF_Send[Length_Send] = {0};		//���ͻ�����
unsigned char BUF_Receive[Length_Receive] = {0};//���ջ�����
unsigned char Position_Send = 0;	   		//�������ݵ�ǰλ��
unsigned char Position_Receive = 0;			//�������ݵ�ǰλ��

unsigned char DATA_Receive[Length_Receive] = {0x00};//���ݴ洢��
unsigned char Flag_Receive = 0;				//������ɱ�־

unsigned char Flag_Init = 0;				//��ʼ����ɱ�־
unsigned char Flag_Error = 0xE0;			//�����־

//------------------------------------------------------------------------------
//Receive
#define Frame_Start_Receive 0x43	//֡ͷ
#define Frame_End_Receive   0x68	//֡β
//Send	
#define Frame_Start_Send	0x65	//֡ͷ
#define Frame_End_Send		0x6E	//֡β
#define Error 			0xE0		//����
#define OK				0x01		//ִ�����
#define Loaded			0x00		//��ʼ�����
#define Loading			0x02		//���ڳ�ʼ��
#define Search			0x03		//����Ѱ����
//Public
#define Find_zero 		0x80		//Ѱ��
#define Flash			0x81		//д��Flash
#define Fine_Positive	0x82		//����΢��
#define Fine_Reverse	0x83		//����΢��
#define Reset			0x14		//��λ
#define Attenuation		0x15		//˥��
#define Stop			0x26		//����ֹͣ
#define Restart			0x27		//����˥����
#define Ratio			0x38		//��ѯÿȦ����
#define Diaplasis		0x39		//��ѯ��λ�����㲽��
#define Default			0xFF		//�ָ���������

//----------------------------------- ��ʼ�� -----------------------------------
/**
* @name      	UART0_Init
* @brief     	���ڳ�ʼ��
* @param    	BAUDRATE ������
* @retval		��
* @attention 	8λ���ݣ�1λֹͣλ������żУ�飬��Ӳ�������� 
*            	��������Ҫ�ɶ�ʱ��1������8λ�Զ���װ��ģʽ����ʽ2������			
*/
void UART0_Init (unsigned long int BAUDRATE)
{
   SCON0 = 0x10;                       // SCON0: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled
                                       //        ninth bits are zeros
                                       //        clear RI0 and TI0 bits
   if (SYSCLK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSCLK/BAUDRATE/2);
      CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
      CKCON |=  0x08;
   } else if (SYSCLK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSCLK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   } else {
      TH1 = -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   }

   TL1 = TH1;                          // init Timer1
   TMOD &= ~0xf0;                      // TMOD: timer 1 in 8-bit autoreload
   TMOD |=  0x20;
   TR1 = 1;                            // START Timer1
   IP |= 0x10;                         // Make UART high priority
   Re = 0;
   De = 0;
   ES0 = 1;                            // Enable UART0 interrupts
}

/**
* @name      	Timer3_Init
* @brief     	��ʱ��3��ʼ��
* @param    	
* @retval	 
* @attention 	���Ƶ��16Hz		
*/
void Timer3_Init (void)
{
	Flag_Error = 0xE0;
   CKCON &= ~0x60;                     	// Timer3 uses SYSCLK/12
   TMR3CN &= ~0x01;

   TMR3RL = -(1029336 / 16);			// Reload value to be used in Timer3
   TMR3 = TMR3RL;                      	// Init the Timer3 register

   TMR3CN = 0x04;                      	// Enable Timer3 in auto-reload mode
   EIE1 |= 0x80;                       	// Timer3 interrupt enabled   
	 P1 |= 0x20;	
}

/**
* @name      	Timer3_Stop
* @brief     	ֹͣ��ʱ��3
* @param    	
* @retval	 
* @attention 			
*/
void Timer3_Stop (void)
{
   TMR3CN &= 0xFB;
	 Flag_Error = 0xE0;
}

/**
* @name      	Loading_Finished
* @brief     	��ʼ�����
* @param    	
* @retval	 
* @attention 			
*/
void Loading_Finished(void)
{
   Flag_Init = 1;
}

//---------------------------------- ���ڷ��� ----------------------------------
/**
* @name      	Send
* @brief     	���ڷ���
* @param    
* @retval		null
* @attention 		
*/
void Send(void)
{
	unsigned char CheckSum = 0;
	unsigned char i = 1;

	while(De);				//æ��ȴ�
	BUF_Send[0] = Frame_Start_Send;		//֡ͷ
	BUF_Send[1] = DATA_Receive[1];		//״̬λ
	
	//��ǰ����λ��
	BUF_Send[2] = Inquire_STEP_Zero() >> 8;	
	BUF_Send[3] = Inquire_STEP_Zero();
	
	for(;i < Length_Send - 2;i++)
		CheckSum += BUF_Send[i];
	BUF_Send[Length_Send - 2] = CheckSum & 0xFF;//У���	
	BUF_Send[Length_Send - 1] = Frame_End_Send;	//֡β
	
	Position_Send = 0;
	De = 1;	
	TI0 = 1;
}

/**
* @name      	Send_Error
* @brief     	���ڷ���
* @param    
* @retval		null
* @attention 		
*/
void Send_Error(unsigned char error)
{
	unsigned char CheckSum = 0;
	unsigned char i = 1;
	
	while(De);							//æ��ȴ�
	BUF_Send[0] = Frame_Start_Send;		//֡ͷ
	BUF_Send[1] = (Error & 0xF0) | (error & 0x0F);	//����
	Flag_Error = BUF_Send[1];			//��¼�������
	
	//��ǰ����λ��
	BUF_Send[2] = Inquire_STEP_Zero() >> 8;	
	BUF_Send[3] = Inquire_STEP_Zero();
	
	for(;i < Length_Send - 2;i++)
		CheckSum += BUF_Send[i];
	BUF_Send[Length_Send - 2] = CheckSum & 0xFF;//У���	
	BUF_Send[Length_Send - 1] = Frame_End_Send;	//֡β
	
	Position_Send = 0;
	De = 1;	
	TI0 = 1;
}

/**
* @name      	Send_Ratio
* @brief     	���ڷ���ÿȦ����
* @param    
* @retval		null
* @attention 		
*/
void Send_Ratio(void)
{
	unsigned char CheckSum = 0;
	unsigned char i = 1;

	while(De);						//æ��ȴ�
	BUF_Send[0] = Frame_Start_Send;	//֡ͷ
	BUF_Send[1] = Ratio;			//���ٱ�
	
	BUF_Send[2] = Read_Step_Default_Flash() >> 8;
	BUF_Send[3] = Read_Step_Default_Flash();
	
	for(;i < Length_Send - 2;i++)
		CheckSum += BUF_Send[i];
	BUF_Send[Length_Send - 2] = CheckSum & 0xFF;//У���
	BUF_Send[Length_Send - 1] = Frame_End_Send;	//֡β
	
	Position_Send = 0;
	De = 1;	
	TI0 = 1;
}

/**
* @name      	Send_Diaplasis
* @brief     	���ڷ��͸�λ��
* @param    
* @retval		null
* @attention 		
*/
void Send_Diaplasis(void)
{
	unsigned char CheckSum = 0;
	unsigned char i = 1;

	while(De);						//æ��ȴ�
	BUF_Send[0] = Frame_Start_Send;	//֡ͷ
	BUF_Send[1] = Diaplasis;		//��λ��
	
	BUF_Send[2] = Read_Step_Flash() >> 8;
	BUF_Send[3] = Read_Step_Flash();
	
	for(;i < Length_Send - 2;i++)
		CheckSum += BUF_Send[i];
	BUF_Send[Length_Send - 2] = CheckSum & 0xFF;//У���
	BUF_Send[Length_Send - 1] = Frame_End_Send;	//֡β
	
	Position_Send = 0;
	De = 1;	
	TI0 = 1;
}

/**
* @name      	Execution_Completed
* @brief     	ִ����ɣ��ش�����
* @param    	
* @retval		null
* @attention 		
*/
void Execution_Completed(void)
{
	DATA_Receive[1] = OK;
	Send();
}


//---------------------------------- ���ڽ��� ----------------------------------
/**
* @name      	Receive
* @brief     	���ڽ���
* @param    
* @retval	-1��δ���յ�����
*			0 ������֡����
*			1 ������֡��ȷ
*			2 �������ص�һ֡����
* @attention 		
*/
signed char Receive(void)
{
	unsigned char CheckSum_Receive = 0,Cycle_Receive = 0;
	unsigned long int Step_Usart = 0;	//�����ݴ�
	if(!Flag_Receive) return -1;		//δ���յ����ݣ��˳�
	Flag_Receive = 0;					//���������ɱ�־
	for(Cycle_Receive = 0;Cycle_Receive < Length_Receive;Cycle_Receive++){	//��������
		DATA_Receive[Cycle_Receive] = BUF_Receive[Cycle_Receive];
		BUF_Receive[Cycle_Receive] = 0;
	}
	
//************ �������� *****************	
	for(Cycle_Receive = 1;Cycle_Receive < Length_Receive - 2;Cycle_Receive++)	//����У���
		CheckSum_Receive += DATA_Receive[Cycle_Receive];
	if(DATA_Receive[0] != Frame_Start_Receive 									//֡ͷ����
		|| DATA_Receive[Length_Receive - 1] != Frame_End_Receive	//֡β����
		|| DATA_Receive[Length_Receive - 2] != CheckSum_Receive)	//У��ʹ���
		{
			Send_Error(0x06);	//����֡���󣬻ش� ����6
			return 0;			//�˳�
		}			

	Step_Usart = ((DATA_Receive[2] & 0x00ff) << 8) | DATA_Receive[3];	//��ȡ����
		
	//ִ�ж�Ӧָ��
	switch(DATA_Receive[1]){
		//-------------- ���ش�һ֡���� ----------------------
		case Default:			//�ָ���������
			Flash_Reset();
			RSTSRC |= 0x10;					//����
		case Restart:			//����˥����
			RSTSRC |= 0x10;					//����	
		case Ratio:				//��ѯÿȦ����
			Send_Ratio();
			return 2;
		case Diaplasis:			//�ش���λ��
			Send_Diaplasis();
			return 2;
		
		//---------------- �ش���֡���� ------------------------
		case Stop:				//����ֹͣ
			Send();
			Motor_Stop();
			break;		
		case Find_zero:			//Ѱ��
			Send();
			Timer3_Init();					//����ʵʱ�ش�
			Cycle_Receive = Find_Zero();	//Ѱ��
			if(Cycle_Receive) Send_Error(Cycle_Receive);	//Ѱ������ش��������
			Timer3_Stop();					//�ر�ʵʱ�ش�	
			break;
		case Flash:				//д��Flash
			Send();
			if(Flash_Write(0)){				//д��ʧ�ܣ��˳�
				Send_Error(0x0E);
				return 2;
			}
			Flash_Init();
			break;
		case Fine_Positive:		//����΢��
			Motor_Positive(Step_Usart);
			break;
		case Fine_Reverse:		//����΢��
			Motor_Reverse(Step_Usart);
			break;
		case Reset:				//��λ
			/*//�ж��Ƿ���ҪѰ��
			Step_Usart = Step_Usart + Read_Step_Flash();
			while(Step_Usart >= Read_Step_Default_Flash())
				Step_Usart -= Read_Step_Default_Flash();
			if(Step_Usart <= Inquire_STEP_Zero())
			*/
			Timer3_Init();					//����ʵʱ�ش�
			Cycle_Receive = Find_Zero();	//Ѱ��
			if(Cycle_Receive) Send_Error(Cycle_Receive);	//Ѱ������ش��������
			Timer3_Stop();					//�ر�ʵʱ�ش�
			Delay10ms();					//�ʵ���ʱ���ȴ���һ���������	
			Motor_Positive(Read_Step_Flash());
			break;
		case Attenuation:		//˥��
			Step_Usart = Step_Usart + Read_Step_Flash();
			/*//�ж��Ƿ���ҪѰ��
			while(Step_Usart >= Read_Step_Default_Flash())
				Step_Usart -= Read_Step_Default_Flash();
			if(Step_Usart <= Inquire_STEP_Zero())
			*/
			Timer3_Init();					//����ʵʱ�ش�
			Cycle_Receive = Find_Zero();	//Ѱ��
			if(Cycle_Receive) Send_Error(Cycle_Receive);	//Ѱ������ش��������
			Timer3_Stop();					//�ر�ʵʱ�ش�
			Delay10ms();					//�ʵ���ʱ���ȴ���һ���������	
			Motor_Positive((unsigned int)Step_Usart - Inquire_STEP_Zero());
			break;
		default:				//����ָ����ش���һ֡����
			Send_Error(0x07);	//����֡���󣬻ش� ����7
			return 2;
			break;
	}
	return 1;
}

/**
* @name      	Receive_Timer3
* @brief     	���ڽ���(��ʱ��3ר��)
* @param    
* @retval	 		
* @attention 		
*/
void Receive_Timer3(void)
{
	unsigned char CheckSum_Receive = 0,Cycle_Receive = 0;
	unsigned long int Step_Usart = 0;	//�����ݴ�
	if(!Flag_Receive) return;			//δ���յ����ݣ��˳�
	Flag_Receive = 0;					//���������ɱ�־
	for(Cycle_Receive = 0;Cycle_Receive < Length_Receive;Cycle_Receive++){		//��������
		DATA_Receive[Cycle_Receive] = BUF_Receive[Cycle_Receive];
		BUF_Receive[Cycle_Receive] = 0;
	}
	
//************ �������� *****************	
	for(Cycle_Receive = 1;Cycle_Receive < Length_Receive - 2;Cycle_Receive++)	//����У���
		CheckSum_Receive += DATA_Receive[Cycle_Receive];
	if(DATA_Receive[0] != Frame_Start_Receive 						//֡ͷ����
		|| DATA_Receive[Length_Receive - 1] != Frame_End_Receive	//֡β����
		|| DATA_Receive[Length_Receive - 2] != CheckSum_Receive)	//У��ʹ���
		{
			Send_Error(0x06);	//����֡���󣬻ش� ����6
			return ;			//���˳�
		}

	//ִ�ж�Ӧָ��
	switch(DATA_Receive[1]){
		case Ratio:				//��ѯÿȦ����
			Send_Ratio();
			break;
		case Diaplasis:			//�ش���λ��
			Send_Diaplasis();
			break;
		
		case Default:			//�ָ���������
			Flash_Reset();
			RSTSRC |= 0x10;					//����
			break;
		case Restart:			//����˥����
			RSTSRC |= 0x10;					//���� 
			break;		
		
		//---------------- ���²���ָ���Ӧ���ش� ���ڳ�ʼ��/Ѱ�� ------------------------
		case Stop:				//����ֹͣ
		case Find_zero:			//Ѱ��
		case Flash:				//д��Flash
		case Fine_Positive:		//����΢��
		case Fine_Reverse:		//����΢��
		case Reset:				//��λ
		case Attenuation:		//˥��
			if(!Flag_Init)DATA_Receive[1] = Loading;//���ڳ�ʼ����
				else DATA_Receive[1] = Search;			//����Ѱ����
				if((Flag_Error != 0xE0) && (Flag_Error != 0xE6)) Send_Error(Flag_Error);	
				else Send();
				break;
		
		//---------------- ��ָ��ش� 0xE7 ������� ------------------------
		default:				//����ָ��ش�
			Send_Error(0x07);				//��ָ��ش� 0xE7 �������
			break;
	}
}

//------------------------------------ �ж� ------------------------------------
/**
* @name      	UATR0_ISR
* @brief     	�����ж�
* @param    
* @retval	 
* @attention 	��Ҫ��������־λ 			
*/
void UATR0_ISR(void)  interrupt 4
{		
	if(RI0){
		BUF_Receive[Position_Receive++] = SBUF0;
		if(Position_Receive >= Length_Receive){
			Position_Receive = 0;
			Flag_Receive = 1;
		}
		RI0 = 0;
	}
	if(TI0){
		if(Position_Send < Length_Send){
			SBUF0 = BUF_Send[Position_Send++];
		}
		else{
			Position_Send = 0;
			De = 0;
		}
		TI0 = 0; 
	}
}

/**
* @name      	Timer3_ISR
* @brief     	��ʱ��3�ж�
* @param    
* @retval	 
* @attention 			
*/
void Timer3_ISR(void) interrupt 14
{
	Receive_Timer3();
	TMR3CN &= 0x3F;			//���������
}

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///