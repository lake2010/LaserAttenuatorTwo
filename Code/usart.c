/*******************************************************************************
* @file      	usart.c
* @author    	Chen
* @version		2.1.2
* @date      	2018/06/06
* @brief     	串口&定时器3
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
* @Update		增加：接收数据帧错误，回传 0xE6 错误代码		 
*******************************************************************************/

#include "c8051f310.h"
#include "usart.h"
#include "flash.h"
#include "steep_motor.h"
#include "sys_Init.h"
#include "usart.h"

#define SYSCLK          24500000       	// SYSCLK frequency in Hz	频率

//------------------------------------------------------------------------------
sfr16 TMR3RL   = 0x92;                 // Timer3 reload value   Timer3重载值
sfr16 TMR3     = 0x94;                 // Timer3 counter	Timer3计数器


//------------------------------------------------------------------------------
sbit Re = P3^1;		//接收使能 低电平有效
sbit De = P3^2;		//发送使能 高电平有效

//------------------------------------------------------------------------------
#define Length_Send 6							//发送缓存区长度
#define Length_Receive 6						//接收缓存区长度
unsigned char BUF_Send[Length_Send] = {0};		//发送缓存区
unsigned char BUF_Receive[Length_Receive] = {0};//接收缓存区
unsigned char Position_Send = 0;	   		//接收数据当前位置
unsigned char Position_Receive = 0;			//发送数据当前位置

unsigned char DATA_Receive[Length_Receive] = {0x00};//数据存储区
unsigned char Flag_Receive = 0;				//接收完成标志

unsigned char Flag_Init = 0;				//初始化完成标志
unsigned char Flag_Error = 0xE0;			//错误标志

//------------------------------------------------------------------------------
//Receive
#define Frame_Start_Receive 0x43	//帧头
#define Frame_End_Receive   0x68	//帧尾
//Send	
#define Frame_Start_Send	0x65	//帧头
#define Frame_End_Send		0x6E	//帧尾
#define Error 			0xE0		//错误
#define OK				0x01		//执行完成
#define Loaded			0x00		//初始化完成
#define Loading			0x02		//正在初始化
#define Search			0x03		//正在寻零中
//Public
#define Find_zero 		0x80		//寻零
#define Flash			0x81		//写入Flash
#define Fine_Positive	0x82		//正向微调
#define Fine_Reverse	0x83		//反向微调
#define Reset			0x14		//复位
#define Attenuation		0x15		//衰减
#define Stop			0x26		//紧急停止
#define Restart			0x27		//重启衰减器
#define Ratio			0x38		//查询每圈步数
#define Diaplasis		0x39		//查询复位点距零点步数
#define Default			0xFF		//恢复出厂设置

//----------------------------------- 初始化 -----------------------------------
/**
* @name      	UART0_Init
* @brief     	串口初始化
* @param    	BAUDRATE 波特率
* @retval		无
* @attention 	8位数据，1位停止位，无奇偶校验，无硬件流控制 
*            	波特率需要由定时器1工作在8位自动重装载模式（方式2）产生			
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
* @brief     	定时器3初始化
* @param    	
* @retval	 
* @attention 	检测频率16Hz		
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
* @brief     	停止定时器3
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
* @brief     	初始化完成
* @param    	
* @retval	 
* @attention 			
*/
void Loading_Finished(void)
{
   Flag_Init = 1;
}

//---------------------------------- 串口发送 ----------------------------------
/**
* @name      	Send
* @brief     	串口发送
* @param    
* @retval		null
* @attention 		
*/
void Send(void)
{
	unsigned char CheckSum = 0;
	unsigned char i = 1;

	while(De);				//忙则等待
	BUF_Send[0] = Frame_Start_Send;		//帧头
	BUF_Send[1] = DATA_Receive[1];		//状态位
	
	//当前绝对位置
	BUF_Send[2] = Inquire_STEP_Zero() >> 8;	
	BUF_Send[3] = Inquire_STEP_Zero();
	
	for(;i < Length_Send - 2;i++)
		CheckSum += BUF_Send[i];
	BUF_Send[Length_Send - 2] = CheckSum & 0xFF;//校验和	
	BUF_Send[Length_Send - 1] = Frame_End_Send;	//帧尾
	
	Position_Send = 0;
	De = 1;	
	TI0 = 1;
}

/**
* @name      	Send_Error
* @brief     	串口发送
* @param    
* @retval		null
* @attention 		
*/
void Send_Error(unsigned char error)
{
	unsigned char CheckSum = 0;
	unsigned char i = 1;
	
	while(De);							//忙则等待
	BUF_Send[0] = Frame_Start_Send;		//帧头
	BUF_Send[1] = (Error & 0xF0) | (error & 0x0F);	//错误
	Flag_Error = BUF_Send[1];			//记录错误代码
	
	//当前绝对位置
	BUF_Send[2] = Inquire_STEP_Zero() >> 8;	
	BUF_Send[3] = Inquire_STEP_Zero();
	
	for(;i < Length_Send - 2;i++)
		CheckSum += BUF_Send[i];
	BUF_Send[Length_Send - 2] = CheckSum & 0xFF;//校验和	
	BUF_Send[Length_Send - 1] = Frame_End_Send;	//帧尾
	
	Position_Send = 0;
	De = 1;	
	TI0 = 1;
}

/**
* @name      	Send_Ratio
* @brief     	串口发送每圈步数
* @param    
* @retval		null
* @attention 		
*/
void Send_Ratio(void)
{
	unsigned char CheckSum = 0;
	unsigned char i = 1;

	while(De);						//忙则等待
	BUF_Send[0] = Frame_Start_Send;	//帧头
	BUF_Send[1] = Ratio;			//减速比
	
	BUF_Send[2] = Read_Step_Default_Flash() >> 8;
	BUF_Send[3] = Read_Step_Default_Flash();
	
	for(;i < Length_Send - 2;i++)
		CheckSum += BUF_Send[i];
	BUF_Send[Length_Send - 2] = CheckSum & 0xFF;//校验和
	BUF_Send[Length_Send - 1] = Frame_End_Send;	//帧尾
	
	Position_Send = 0;
	De = 1;	
	TI0 = 1;
}

/**
* @name      	Send_Diaplasis
* @brief     	串口发送复位点
* @param    
* @retval		null
* @attention 		
*/
void Send_Diaplasis(void)
{
	unsigned char CheckSum = 0;
	unsigned char i = 1;

	while(De);						//忙则等待
	BUF_Send[0] = Frame_Start_Send;	//帧头
	BUF_Send[1] = Diaplasis;		//复位点
	
	BUF_Send[2] = Read_Step_Flash() >> 8;
	BUF_Send[3] = Read_Step_Flash();
	
	for(;i < Length_Send - 2;i++)
		CheckSum += BUF_Send[i];
	BUF_Send[Length_Send - 2] = CheckSum & 0xFF;//校验和
	BUF_Send[Length_Send - 1] = Frame_End_Send;	//帧尾
	
	Position_Send = 0;
	De = 1;	
	TI0 = 1;
}

/**
* @name      	Execution_Completed
* @brief     	执行完成，回传数据
* @param    	
* @retval		null
* @attention 		
*/
void Execution_Completed(void)
{
	DATA_Receive[1] = OK;
	Send();
}


//---------------------------------- 串口接收 ----------------------------------
/**
* @name      	Receive
* @brief     	串口接收
* @param    
* @retval	-1：未接收到数据
*			0 ：数据帧错误
*			1 ：数据帧正确
*			2 ：仅返回第一帧数据
* @attention 		
*/
signed char Receive(void)
{
	unsigned char CheckSum_Receive = 0,Cycle_Receive = 0;
	unsigned long int Step_Usart = 0;	//步数暂存
	if(!Flag_Receive) return -1;		//未接收到数据，退出
	Flag_Receive = 0;					//清除接收完成标志
	for(Cycle_Receive = 0;Cycle_Receive < Length_Receive;Cycle_Receive++){	//拷贝数据
		DATA_Receive[Cycle_Receive] = BUF_Receive[Cycle_Receive];
		BUF_Receive[Cycle_Receive] = 0;
	}
	
//************ 处理数据 *****************	
	for(Cycle_Receive = 1;Cycle_Receive < Length_Receive - 2;Cycle_Receive++)	//计算校验和
		CheckSum_Receive += DATA_Receive[Cycle_Receive];
	if(DATA_Receive[0] != Frame_Start_Receive 									//帧头错误
		|| DATA_Receive[Length_Receive - 1] != Frame_End_Receive	//帧尾错误
		|| DATA_Receive[Length_Receive - 2] != CheckSum_Receive)	//校验和错误
		{
			Send_Error(0x06);	//数据帧错误，回传 错误6
			return 0;			//退出
		}			

	Step_Usart = ((DATA_Receive[2] & 0x00ff) << 8) | DATA_Receive[3];	//读取步数
		
	//执行对应指令
	switch(DATA_Receive[1]){
		//-------------- 仅回传一帧数据 ----------------------
		case Default:			//恢复出厂设置
			Flash_Reset();
			RSTSRC |= 0x10;					//重启
		case Restart:			//重启衰减器
			RSTSRC |= 0x10;					//重启	
		case Ratio:				//查询每圈步数
			Send_Ratio();
			return 2;
		case Diaplasis:			//回传复位点
			Send_Diaplasis();
			return 2;
		
		//---------------- 回传两帧数据 ------------------------
		case Stop:				//紧急停止
			Send();
			Motor_Stop();
			break;		
		case Find_zero:			//寻零
			Send();
			Timer3_Init();					//开启实时回传
			Cycle_Receive = Find_Zero();	//寻零
			if(Cycle_Receive) Send_Error(Cycle_Receive);	//寻零出错，回传错误代码
			Timer3_Stop();					//关闭实时回传	
			break;
		case Flash:				//写入Flash
			Send();
			if(Flash_Write(0)){				//写入失败，退出
				Send_Error(0x0E);
				return 2;
			}
			Flash_Init();
			break;
		case Fine_Positive:		//正向微调
			Motor_Positive(Step_Usart);
			break;
		case Fine_Reverse:		//反向微调
			Motor_Reverse(Step_Usart);
			break;
		case Reset:				//复位
			/*//判断是否需要寻零
			Step_Usart = Step_Usart + Read_Step_Flash();
			while(Step_Usart >= Read_Step_Default_Flash())
				Step_Usart -= Read_Step_Default_Flash();
			if(Step_Usart <= Inquire_STEP_Zero())
			*/
			Timer3_Init();					//开启实时回传
			Cycle_Receive = Find_Zero();	//寻零
			if(Cycle_Receive) Send_Error(Cycle_Receive);	//寻零出错，回传错误代码
			Timer3_Stop();					//关闭实时回传
			Delay10ms();					//适当延时，等待上一步操作完毕	
			Motor_Positive(Read_Step_Flash());
			break;
		case Attenuation:		//衰减
			Step_Usart = Step_Usart + Read_Step_Flash();
			/*//判断是否需要寻零
			while(Step_Usart >= Read_Step_Default_Flash())
				Step_Usart -= Read_Step_Default_Flash();
			if(Step_Usart <= Inquire_STEP_Zero())
			*/
			Timer3_Init();					//开启实时回传
			Cycle_Receive = Find_Zero();	//寻零
			if(Cycle_Receive) Send_Error(Cycle_Receive);	//寻零出错，回传错误代码
			Timer3_Stop();					//关闭实时回传
			Delay10ms();					//适当延时，等待上一步操作完毕	
			Motor_Positive((unsigned int)Step_Usart - Inquire_STEP_Zero());
			break;
		default:				//其他指令仅回传第一帧数据
			Send_Error(0x07);	//数据帧错误，回传 错误7
			return 2;
			break;
	}
	return 1;
}

/**
* @name      	Receive_Timer3
* @brief     	串口接收(定时器3专用)
* @param    
* @retval	 		
* @attention 		
*/
void Receive_Timer3(void)
{
	unsigned char CheckSum_Receive = 0,Cycle_Receive = 0;
	unsigned long int Step_Usart = 0;	//步数暂存
	if(!Flag_Receive) return;			//未接收到数据，退出
	Flag_Receive = 0;					//清除接收完成标志
	for(Cycle_Receive = 0;Cycle_Receive < Length_Receive;Cycle_Receive++){		//拷贝数据
		DATA_Receive[Cycle_Receive] = BUF_Receive[Cycle_Receive];
		BUF_Receive[Cycle_Receive] = 0;
	}
	
//************ 处理数据 *****************	
	for(Cycle_Receive = 1;Cycle_Receive < Length_Receive - 2;Cycle_Receive++)	//计算校验和
		CheckSum_Receive += DATA_Receive[Cycle_Receive];
	if(DATA_Receive[0] != Frame_Start_Receive 						//帧头错误
		|| DATA_Receive[Length_Receive - 1] != Frame_End_Receive	//帧尾错误
		|| DATA_Receive[Length_Receive - 2] != CheckSum_Receive)	//校验和错误
		{
			Send_Error(0x06);	//数据帧错误，回传 错误6
			return ;			//均退出
		}

	//执行对应指令
	switch(DATA_Receive[1]){
		case Ratio:				//查询每圈步数
			Send_Ratio();
			break;
		case Diaplasis:			//回传复位点
			Send_Diaplasis();
			break;
		
		case Default:			//恢复出厂设置
			Flash_Reset();
			RSTSRC |= 0x10;					//重启
			break;
		case Restart:			//重启衰减器
			RSTSRC |= 0x10;					//重启 
			break;		
		
		//---------------- 以下操作指令不响应，回传 正在初始化/寻零 ------------------------
		case Stop:				//紧急停止
		case Find_zero:			//寻零
		case Flash:				//写入Flash
		case Fine_Positive:		//正向微调
		case Fine_Reverse:		//反向微调
		case Reset:				//复位
		case Attenuation:		//衰减
			if(!Flag_Init)DATA_Receive[1] = Loading;//正在初始化中
				else DATA_Receive[1] = Search;			//正在寻零中
				if((Flag_Error != 0xE0) && (Flag_Error != 0xE6)) Send_Error(Flag_Error);	
				else Send();
				break;
		
		//---------------- 空指令，回传 0xE7 错误代码 ------------------------
		default:				//其他指令回传
			Send_Error(0x07);				//空指令，回传 0xE7 错误代码
			break;
	}
}

//------------------------------------ 中断 ------------------------------------
/**
* @name      	UATR0_ISR
* @brief     	串口中断
* @param    
* @retval	 
* @attention 	需要软件清除标志位 			
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
* @brief     	定时器3中断
* @param    
* @retval	 
* @attention 			
*/
void Timer3_ISR(void) interrupt 14
{
	Receive_Timer3();
	TMR3CN &= 0x3F;			//清除溢出标记
}

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///