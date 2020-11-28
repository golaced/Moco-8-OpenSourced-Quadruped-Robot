#include "stm32f10x.h"
#include "key.h"
#include "delay.h"
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
		
//按键IO初始化函数
void keyInit(void) 
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin  = KEY_SEL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = KEY_1 | KEY_2 |KEY_3 | KEY_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ; //上拉输入 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

u8 key_o[5];
void KEY_Scan(float dt)
{	 
	static u32 cnt;
	u8 sel;
	
  key_o[4]=READ_KEY_SEL();
	key_o[0]=READ_KEY_1();
	key_o[1]=READ_KEY_2();
	key_o[2]=READ_KEY_3();
	key_o[3]=READ_KEY_4();
}












