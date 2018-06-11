/*******************************************************************************
* @file      	main.c
* @author    	Chen
* @version   	2.1.1
* @date      	2018/04/28
* @brief     	激光衰减器		   
********************************************************************************
* @Email     	Chen.0810@outlook.com
* @Github    	https://github.com/Chen-0810
********************************************************************************
* @attention	1 Table = 4 Spacebar.
*            	上电即复位 。
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
sfr16 DP       = 0x82;                 	// data pointer		数据指针
sfr16 PCA0CP0  = 0xfb;                 	// PCA0 Module 0 Capture/Compare		
sfr16 PCA0CP1  = 0xe9;                 	// PCA0 Module 1 Capture/Compare		
sfr16 PCA0CP2  = 0xeb;                 	// PCA0 Module 2 Capture/Compare		
sfr16 PCA0CP3  = 0xed;                 	// PCA0 Module 3 Capture/Compare		
sfr16 PCA0CP4  = 0xfd;                 	// PCA0 Module 4 Capture/Compare			
sfr16 PCA0     = 0xf9;                 	// PCA0 counter		PCA0计数器

//------------------------------------------------------------------------------
#define Failed_passbacks	5	//失败回传次数

//----------------------------------- 主函数 -----------------------------------
/**
* @name      	main
* @brief     	主函数
* @param    
* @retval	 
* @attention 			
*/
void main(void){
	unsigned char Return = 0;
	unsigned char Count = 0;
//-------------------- 初始化 ----------------------------	
	sys_Init();			//系统初始化
	Motor_Init();		//电机初始化
	UART0_Init(115200);	//初始化串口，波特率115200bps
 	EA = 1;				//使能全局中断
	
	Timer3_Init();		//开启实时回传指令
	
	if(Flash_Init() == 1){			//首次开机
		Return = Motor_Boot();		//测量每圈步数
		if(Return) {				//测量失败，返回对应错误代码x次并重启
			for(Count = 0;Count < Failed_passbacks;Count++){		
				Send_Error(Return);	//返回错误代码	
				Delay500ms();Delay500ms();
			}
			Restart();			//重启
		}
		else Send_Ratio();		//测量成功，回传每圈步数
	}
	else if(!Flash_Init()) {	//flash错误，回传错误代码
		while(1){				//回传错误指令且等待上位机指令（重启/重置）
			Send_Error(0x0D);	//flash错误，回传错误代码	
			Delay500ms();Delay500ms();
			}
		}
		else {					//读取flash成功	
			Return = Init();	//上电正常初始化
			if(Return){ 		//初始化失败，返回对应错误代码x次并重启
				for(Count = 0;Count < Failed_passbacks;Count++){		
					Send_Error(Return);			//返回错误代码
					Delay500ms();Delay500ms();
				}
				Restart();		//重启
			}
			Motor_Positive(Read_Step_Flash());	//复位
		}		
			
	Delay10ms();	
	while(Inquire_STEP());		//等待复位指令执行完毕		
	Timer3_Stop();				//关闭实时回传指令
	Loading_Finished();			//初始化完成
	Send();						//回传初始化完成指令

//------------------- 正常工作 ----------------------------
	while(1){
		if(Receive() == 1)		//接收到正确数据
			Return = 1;
		if(!Inquire_STEP() && (Return == 1)){	//接收到正确数据且执行完成
			Motor_Stop();				//刹车
			Return  = 0;
			Execution_Completed();		//回传第二帧数据，表明执行完毕
		}
	}
}

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///
