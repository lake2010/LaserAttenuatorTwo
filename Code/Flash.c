/*******************************************************************************
* @file      	Flash.c
* @author    	Chen
* @version  	2.0.1
* @date      	2018/03/01
* @brief     	flash存取  
********************************************************************************
* @Email     	Chen.0810@outlook.com
* @Github    	https://github.com/Chen-0810
********************************************************************************
* @attention	1 Table = 4 Spacebar.  
*            	FLASH规模16384字节，其中0x3E00~0x3FFF保留
*            	本程序使用自0x3A00后的9字节，擦除扇区0x3C00~0x3E00
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
#define Length_Flash  9  		//flash数据帧长度
#define Flash_Start   0x3A00	//flash开始存储位置
#define Frame_Start_0 0x43		//帧头0
#define Frame_Start_1 0x68		//帧头1
#define Frame_End_0   0x65		//帧尾0
#define Frame_End_1   0x6E		//帧尾1

//------------------------------------------------------------------------------
unsigned int Step_Zero_Flash = 0;  			//flash读出来的步数
unsigned int Step_Default_Flash = 0;		//默认一圈3200步

//----------------------------------- 初始化 -----------------------------------
/**
* @name			Flash_Init
* @brief		Flash初始化
* @param     
* @retval		0:flash错误
*				1:首次开机
*				2:OK
* @attention 			
*/
unsigned char Flash_Init(void)
{	
	signed int Cache = Flash_Read();
	if(Cache < -1)		//flash错误
		return 0;
	if(Cache == -1){	//首次开机
		Step_Zero_Flash = 0;
		return 1;
	}
	else Step_Zero_Flash = Cache;
	return 2;
}

//----------------------------- 操作Flash -----------------------------
/**
* @name			Read_Step_Flash
* @brief		读取flash中步数
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
* @brief		读取flash中每圈步数
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
* @brief		写入Flash
* @param     
* @retval		0:OK
*				1:写入失败
* @attention 			
*/
unsigned char Flash_Write(unsigned int Step_Default)
{
	unsigned char flash[Length_Flash] = 
		{Frame_Start_0,Frame_Start_1,0x00,0x00,0x00,0x00,0x00,Frame_End_0,Frame_End_1};
									//预写入flash序列值 
	unsigned char Position_Flash;	//flash当前位置
  	unsigned char xdata *Flash_write;//写指针，
	unsigned char code *Flash_read;	//读指针，
  	unsigned char EA_Save;  		//暂存中断标志
	unsigned char CheckSum = 0;		//校验和
	unsigned int 	Step = 0;		//暂存当前位置步数
		
	if(Step_Default == 0)	{		//写入复位点，不涉及写入每圈步数，应读出flash中存储的每圈步数
		Flash_read = (unsigned char code *) Flash_Start;	
		flash[2] = *(Flash_read + 2);
		flash[3] = *(Flash_read + 3);
		
		Step = Inquire_STEP_Zero();	//获取当前步数
		flash[4] = (Step & 0xFF00) >> 8;		
		flash[5] = (Step & 0x00FF);		
	}
	else {							//第一次上电，往flash写入每圈步数
		flash[2] = Step_Default >> 8;
		flash[3] = Step_Default;
		Step_Default_Flash = Step_Default;
		flash[4] = 0;		
		flash[5] = 0;	
	}
	
	//计算校验和	
	for(Position_Flash = 2;Position_Flash < Length_Flash - 3;Position_Flash++){
		CheckSum += flash[Position_Flash];
	}
	flash[Position_Flash] = CheckSum;
	
	RSTSRC |= 0x02;		//使能片内VDD监控                                    	
	PCA0MD &= ~0x40;	//禁用看门狗定时器 										  
	EA_Save = EA;		//保存中断 
  	EA = 0;				//禁用中断(预防性)
    
	//擦除扇区	
	Flash_write = (unsigned char xdata *) Flash_Start;//指针指向目标位置，准备擦除扇区
	PSCTL = 0x03;                         	//准备擦除flash扇区 512字节
	FLKEY = 0xA5;                         	//写入第一个FLASH关键码0xA5
	FLKEY = 0xF1;                         	//写入第二个FLASH关键码0xF1
	*Flash_write = 0x00;                  	//写入任意值以擦除该页面
	PSCTL = 0;                            	//软件清除PSWE位

	//写入	
	EA= 0;					//禁止中断
 	Flash_write = Flash_Start;            	//指针指向目标位置，准备写入数据
	PSCTL = 0x01;                         	//置位PSWE,写Flash
	for(Position_Flash = 0;Position_Flash < Length_Flash;Position_Flash++) {    //开始写入Flash
		FLKEY = 0xA5;                   	//写入第一个FLASH关键码0xA5
		FLKEY = 0xF1;                   	//写入第二个FLASH关键码0xF1
  	*Flash_write = flash[Position_Flash];   //写入数据
  	Flash_write++;
	}
 	PSCTL = 0x00;                         	//
 	EA = EA_Save;                         	//恢复全局中断
	
	if(!Step_Default && Flash_Read() != Step) return 1;		//写入失败
	return 0;
}

/**
* @name			Flash_Read
* @brief		读取Flash
* @param     
* @retval		-2:帧头或帧尾或校验有误
*				-1:首次开机
*				xx:标定步数
* @attention 			
*/
signed int Flash_Read(void)
{
	unsigned char flash[Length_Flash]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//预写入flash序列值
	unsigned char code *Flash_read; 
	unsigned char Position_Flash;	//flash当前位置
	unsigned char CheckSum = 0;		//校验和
	unsigned char Flag_Boot = 0;	//首次开机标志
	
	//读出flash
	Flash_read = (unsigned char code *) Flash_Start;
	for(Position_Flash = 0;Position_Flash < Length_Flash;Position_Flash++){	
		flash[Position_Flash] = *(Flash_read + Position_Flash);
		if(flash[Position_Flash] == 0xFF) Flag_Boot++;
	}	
	
	if(Flag_Boot == Length_Flash) return -1;	//首次开机
		
	//计算校验和
	for(Position_Flash = 2;Position_Flash < Length_Flash - 3;Position_Flash++){
		CheckSum += flash[Position_Flash];
	}
	if((flash[0] != Frame_Start_0) || (flash[1] != Frame_Start_1) 
		|| (flash[Length_Flash - 2] != Frame_End_0) || (flash[Length_Flash - 1] != Frame_End_1)
		|| (CheckSum != flash[Length_Flash - 3])) 
		return -2;				//帧头或帧尾或校验有误，退出
	
	Step_Default_Flash = ((flash[2] & 0x00FF) << 8) | flash[3];		//flash中存储的每圈步数
	
	return ((flash[4] & 0x00FF) << 8) | flash[5];	//返回标定步数
}

/**
* @name			Flash_Reset
* @brief		恢复出厂设置，重置Flash数据
* @param     
* @retval		1:OK
* @attention 			
*/
unsigned char Flash_Reset(void)
{
	unsigned char xdata *Flash_write;	//指针，
  	unsigned char EA_Save;  			//暂存中断标志
	
	RSTSRC |= 0x02;                   	//使能片内VDD监控                                    	
	PCA0MD &= ~0x40;             		//禁用看门狗定时器 										  
	EA_Save = EA;						//保存中断 
  	EA = 0;                             //禁用中断(预防性)
    
	//擦除扇区	
	Flash_write = (unsigned char xdata *) Flash_Start;//指针指向目标位置，准备擦除扇区
	PSCTL = 0x03;                        //准备擦除flash扇区 512字节
	FLKEY = 0xA5;                        //写入第一个FLASH关键码0xA5
	FLKEY = 0xF1;                        //写入第二个FLASH关键码0xF1
	*Flash_write = 0;                    //写入任意值以擦除该页面
	PSCTL = 0;                           //软件清除PSWE位

 	EA = EA_Save;                        //恢复全局中断
	return 1;
}

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///