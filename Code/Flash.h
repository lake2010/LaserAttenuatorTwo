/*******************************************************************************
* @file      	Flash.h
* @author    	Chen
* @version   	2.0.1
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

#ifndef _FLASH_H
#define _FLASH_H_H

extern unsigned char Flash_Write(unsigned int Step_Default); //写入Flash
extern signed int Flash_Read(void);						//读取Flash
extern unsigned char Flash_Init(void);	                //初始化Flash
extern unsigned int Read_Step_Flash(void);				//读取flash中复位点步数
extern unsigned char Flash_Reset(void);                 //重置flash数据
extern unsigned int Read_Step_Default_Flash(void);      //读取flash中每圈步数

#endif

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///