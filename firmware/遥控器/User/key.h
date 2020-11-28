#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly_Remotor
 * 按键驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/   	 

#define KEY_1  		GPIO_Pin_15
#define KEY_2  		GPIO_Pin_14
#define KEY_3   	GPIO_Pin_13
#define KEY_4   	GPIO_Pin_12
#define KEY_SEL   GPIO_Pin_8

#define READ_KEY_1()  	GPIO_ReadInputDataBit(GPIOB,KEY_1)	//读取左按键
#define READ_KEY_2()   	GPIO_ReadInputDataBit(GPIOB,KEY_2)	//读取右按键
#define READ_KEY_3()  	GPIO_ReadInputDataBit(GPIOB,KEY_3)	//读取摇杆1按键
#define READ_KEY_4()  	GPIO_ReadInputDataBit(GPIOB,KEY_4)	//读取摇杆1按键
#define READ_KEY_SEL()  !GPIO_ReadInputDataBit(GPIOA,KEY_SEL)	//读取摇杆2按键

//IO初始化
void keyInit(void);

 //按键扫描函数		
void KEY_Scan(float dt);

extern u8 key_o[5];

#endif



