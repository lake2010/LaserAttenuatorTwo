/*******************************************************************************
* @file      	steep_motor.c
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

#include "c8051f310.h"
#include <math.h>
#include "steep_motor.h"
#include "flash.h"
#include "steep_motor.h"
#include "sys_Init.h"
#include "usart.h"

//------------------------------------------------------------------------------
sfr16 TMR2RL   = 0xca;		// Timer2 reload value 	Timer2重载值
sfr16 TMR2     = 0xcc; 		// Timer2 counter	Timer2计数器

sbit PULL = P0^0;			//脉冲
sbit EN =  P0^1;			//使能
sbit DIR = P0^2;			//方向
sbit MS1 = P3^4;			//细分
sbit MS2 = P3^3;			//细分

sbit  Zero = P1^5;			//Zero2 霍尔开关  0 Read

//------------------------------------------------------------------------------
#define Hertz_min 32							//最低速度32hz
#define Hertz_max 1000							//最高速度1600hz 1000
#define Steps_Low_speed 8						//低速步长  8
#define Frequency_conversion_Step_Default 200	//默认变频步长 200步
#define Frequency_conversion (Hertz_max - Hertz_min - 2 * Steps_Low_speed) / Frequency_conversion_Step			

#define Dir_Default 1				//默认初始化方向
//#define Step_Default 3200			//默认一圈3200步
#define Difference 32				//默认差值 100 步

//------------------------------------------------------------------------------
unsigned long int STEP = 0,STEP_Executed = 0;
signed int STEP_Zero = 100;			//相对零点步数
unsigned long int Frequency_conversion_Step = Frequency_conversion_Step_Default;	//实际变频步长

//--------------------------------- 电机初始化 ---------------------------------
/**
* @name      	Motor_Init
* @brief     	步进电机初始化
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
* @brief     	定时器2初始化
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
* @brief     	步进电机驱动初始化
* @param    	
* @retval	 
* @attention 			
*/
void Drive_Init(void)
{
	EN = 1;				//失能
	MS1 = 1;			//默认最大细分 8细分=16细分
	MS2 = 1;
	DIR = Dir_Default;	//方向默认
	EN = 0;
}

//----------------------------------- 初始化 -----------------------------------
/**
* @name       	Init
* @brief     	衰减器第一次寻零成功后初始化
* @param     	null
* @retval	 	0:OK
*				1:霍尔开关常闭故障
*				2:霍尔开关常开故障
*				3:微调故障
* @attention 	此过程继承上一过程频率，走完默认步数后减速停止		
*/
unsigned char Init(void)
{
	unsigned char count = 0;
//第一圈：高速寻零盲检，3圈之内寻零失败则重启
	Delay500ms();				//延时500ms
 	Motor_Positive(Read_Step_Default_Flash() * 3);
	while(!Zero){ 				//启动触发，等待越过触发区
		if(!Inquire_STEP())		//霍尔开关常闭异常（一直触发）
		  return 1;
	}
	while(Zero){   				//等待触发
		if(!Inquire_STEP())		//霍尔开关常开异常（检测不到）
			return 2;
	}
	STEP_Zero = 0;
//---------------------------------------
	STEP = Read_Step_Default_Flash() - Difference;	//设定步数
//---------------------------------------
	while(!Zero){ 				//启动触发，等待越过触发区
		if(!Inquire_STEP())		//霍尔开关常闭异常
		   return 1;
	}
	while(Zero){   				//等待触发
		if(!Inquire_STEP()){ 	//微调
			count ++;
			Motor_Positive(1);
		}
		if(count >= 0.2 * Read_Step_Default_Flash())	//微调异常
			return 3;
	}
	Motor_Stop();
	STEP_Zero = 0;
	return 0;
}

/**
* @name       	Motor_Boot
* @brief      	首次开机，测量每圈步数
* @param     	null
* @retval	 	0:OK
*				1:霍尔开关常闭故障
*				2:霍尔开关常开故障
*				3:微调故障
*				4:记步差值过大
* @attention 			
*/
unsigned char Motor_Boot(void)
{
	unsigned char count = 0,i = 0;
	unsigned int Step_Temporary[4] = 0;		//暂存每圈步数
	unsigned int Step_Default = 3200;		//默认一圈3200步
//高速盲检寻找零点
 	Motor_Positive(Step_Default * 6);
	while(!Zero){ 				//启动触发，等待越过触发区
		if(!Inquire_STEP())		//霍尔开关常闭异常
		  return 1;
	}
	while(Zero){   				//等待触发
		if(!Inquire_STEP())		//霍尔开关常开异常
			return 2;
	}
//高速记步两圈---------------------------------------	
	STEP_Zero = 0;
	STEP = Step_Default * 5;	//设定步数
	for(i = 0;i < 2;i++){
		while(!Zero){ 			//启动触发，等待越过触发区
			if(!Inquire_STEP())	//霍尔开关常闭异常
				return 1;
		}
		while(Zero){   			//等待触发
			if(!Inquire_STEP()) //霍尔开关常开异常
				return 2;		//霍尔开关常开故障
		}
		Step_Temporary[i] = STEP_Zero;	//获取第一圈高速每圈步数
		STEP_Zero = 0;
	}
	if(abs(Step_Temporary[0] - Step_Temporary[1]) > Difference) return 4;//如果记步差值过大，返回4
//第三圈减速过渡---------------------------------------
	//判断是否进入减速阶段
	if(STEP > Frequency_conversion_Step_Default) 
		STEP = (Step_Temporary[0] + Step_Temporary[1]) / 2 - Difference;
	else {						//若进入减速阶段，则减速结束后再开始新一轮操作
		while(Inquire_STEP());
		Motor_Positive((Step_Temporary[0] + Step_Temporary[1]) / 2 - Difference / 2);
	}
	while(!Zero){ 				//启动触发，等待越过触发区
		if(!Inquire_STEP())		//霍尔开关常闭异常
			return 1;
	}
	count = 0;
	while(Zero){   				//等待触发
		if(!Inquire_STEP()){ 	//微调
			count ++;
			Motor_Positive(1);
		}
		if(count >= 0.2 * (Step_Temporary[0] + Step_Temporary[1]) / 2)	//微调异常
			return 3;
	}
	Motor_Stop();
//低速记步两圈---------------------------------------	
	for(i = 2;i < 4;i++){
		STEP_Zero = 0;		
		Motor_Positive((Step_Temporary[0] + Step_Temporary[1]) / 2 - Difference / 2);
		while(!Zero){ 			//启动触发，等待越过触发区
			if(!Inquire_STEP())	//霍尔开关常闭异常
				return 1;
		}
		count = 0;
		while(Zero){   				//等待触发
			if(!Inquire_STEP()){	//微调
				count ++;
				Motor_Positive(1);
			}
			if(count >= 0.2 * (Step_Temporary[0] + Step_Temporary[1]) / 2)	//微调异常
				return 3;
		}
		Motor_Stop();
		Step_Temporary[i] = STEP_Zero;	//获取第圈每圈步数
	}
	if(abs(Step_Temporary[2] - Step_Temporary[3]) > Difference) return 4;//如果高速记步差值过大，返回4
	if(abs(((Step_Temporary[0] + Step_Temporary[1]) / 2) - ((Step_Temporary[2] + Step_Temporary[3]) / 2)) > Difference) return 4;//如果高速记步差值过大，返回4
	Step_Default = (Step_Temporary[0] + Step_Temporary[1] + Step_Temporary[2] + Step_Temporary[3]) / 4 + 0.5;
	STEP_Zero = 0;
	Flash_Write(Step_Default);	
	return 0;
}

/**
* @name       	Find_Zero
* @brief     	寻零
* @param     	null
* @retval	 	1:霍尔开关常闭故障
*				2:霍尔开关常开故障
*				3:微调故障
*				5:多次尝试寻零失败
*				0:OK
* @attention 			
*/
unsigned char Find_Zero(void)
{
	unsigned int count = 0;
	static unsigned char Count_Retry = 0;
	
	Motor_Positive(Read_Step_Default_Flash() - STEP_Zero);
	while(!Zero){ 				//启动触发，等待越过触发区
		if(!Inquire_STEP())		 
		   	return 1;			//霍尔开关常闭故障
	}
	while(Zero){   				//等待触发
		if(!Inquire_STEP()){ 	//微调
			count ++;
			Motor_Positive(1);
		}
		if(count >= 0.2 * Read_Step_Default_Flash())	
			return 3;			//微调异常
	}
	STEP_Zero= 0;
	//判断是否进入减速阶段
	if(STEP <= Steps_Low_speed) 
		Motor_Stop();			//低速状态直接停止
	else {						//高速状态，说明上一步骤异常，减速后尝试重新寻零
		if(STEP > Frequency_conversion_Step_Default) STEP = Frequency_conversion_Step_Default;//开始减速
		while(Inquire_STEP());	//等待执行完毕
		Find_Zero();			//重新寻零
		Count_Retry++;			//累计重试次数
		if(Count_Retry > 2) return 5;
	}
	Count_Retry = 0;
	Motor_Stop();
	return 0;
}

//------------------------------------ 电机 -------------------------------------
/**
* @name       	Motor
* @brief     	步进电机控制
* @param     	方向&步数
* @retval	 	null
* @attention 			
*/
void Motor(unsigned char Dir,unsigned long int Step)
{
	DIR = Dir;				//设定方向
	EN = 0;					//使能
	if(!Step) return;
	//判断步数是否够完成加减速全过程
	if(Step < 2 * Frequency_conversion_Step_Default) Frequency_conversion_Step = Step / 2;	//不够全过程
	else Frequency_conversion_Step = Frequency_conversion_Step_Default;						//够全过程
	STEP = Step;			//设定步数
	STEP_Executed = 0;		//清除已执行步数
	TMR2 = TMR2RL = -(2058672 / Hertz_min); 	//设定定时器2自动重装载值
	TR2 = 1;				//使能定时器2
	EA = 1;					//使能全局中断
}

/**
* @name       	Motor_Positive
* @brief     	步进电机控制
* @param     	正向&步数
* @retval	 	null
* @attention 			
*/
void Motor_Positive(unsigned long int Step)
{
	Motor(Dir_Default,Step);
}

/**
* @name       	Motor_Reverse
* @brief     	步进电机控制
* @param     	反向&步数
* @retval	 	null
* @attention 			
*/
void Motor_Reverse(unsigned long int Step)
{
	Motor(!Dir_Default,Step);
}

/**
* @name       	Motor_Stop
* @brief     	步进电机刹车
* @param     	null
* @retval	 	null
* @attention 			
*/
void Motor_Stop(void)
{
	if(STEP <= Steps_Low_speed) {
		EN = 0;			//使能
		TR2 = 0;		//使能定时器2
		STEP = 0;
	}
	else if(STEP >= Frequency_conversion_Step && STEP_Executed >= Frequency_conversion_Step )	//全速中停止
					STEP = Frequency_conversion_Step;	
	else if(STEP < Frequency_conversion_Step);			//减速中
	else if(STEP_Executed < Frequency_conversion_Step)	//加速中
					STEP = STEP_Executed;
	else{
//----------------------------------------	
		EN = 0;			//使能
		TR2 = 0;		//使能定时器2
		STEP = 0;
	}
}

//------------------------------------ 查询 -------------------------------------
/**
* @name      	Inquire_STEP
* @brief     	查询剩余步数
* @param    	
* @retval	 	剩余步数
* @attention 			
*/
unsigned long int Inquire_STEP(void)
{
	return STEP;
}

/**
* @name      	Inquire_STEP
* @brief     	查询相对零点步数
* @param    	
* @retval	 	相对零点步数
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
* @brief     	相对零点步数清零
* @param    	
* @retval	 
* @attention 			
*/
void Clear_STEP_Zero(void)
{
	STEP_Zero = 0;
}

//------------------------------------ 中断 -------------------------------------
/**
* @name      	Timer2_ISR
* @brief     	定时器2中断
* @param    
* @retval	 
* @attention 			
*/
void Timer2_ISR(void) interrupt 5
{
	PULL = 0;
	STEP--;
	STEP_Executed++;	
	if(!STEP) TR2 = 0;  		//步数走完，关闭定时器2
	
	if(DIR == Dir_Default) STEP_Zero++;		//相对零点步数与默认方向同向 +1
	else STEP_Zero--;						//相对零点步数与默认方向反向 -1
	
	if(STEP <= Steps_Low_speed || STEP_Executed <= Steps_Low_speed) //低速运行区
		TMR2 = TMR2RL = -(2058672 / Hertz_min);
	if(STEP < Frequency_conversion_Step) 				//减速
		TMR2 = TMR2RL = -(2058672 / (Hertz_min + STEP * Frequency_conversion)); // Reload value to be used in Timer2
	else if(STEP_Executed < Frequency_conversion_Step)	//加速
					TMR2 = TMR2RL = -(2058672 / (Hertz_min + STEP_Executed * Frequency_conversion)); // Reload value to be used in Timer2
	
	PULL = 1;
	TF2H = TF2L = 0;			//清除溢出标记
}

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///