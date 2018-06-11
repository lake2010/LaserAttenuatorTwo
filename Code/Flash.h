/*******************************************************************************
* @file      	Flash.h
* @author    	Chen
* @version   	2.0.1
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

#ifndef _FLASH_H
#define _FLASH_H_H

extern unsigned char Flash_Write(unsigned int Step_Default); //д��Flash
extern signed int Flash_Read(void);						//��ȡFlash
extern unsigned char Flash_Init(void);	                //��ʼ��Flash
extern unsigned int Read_Step_Flash(void);				//��ȡflash�и�λ�㲽��
extern unsigned char Flash_Reset(void);                 //����flash����
extern unsigned int Read_Step_Default_Flash(void);      //��ȡflash��ÿȦ����

#endif

///************************  COPYRIGHT (C) 2018 Chen ************************///
///******************************* END OF FILE ******************************///