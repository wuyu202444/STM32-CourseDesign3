/***********************************************************
* 模块名称：四位共阳极数码管驱动模块
* 文件名  ：BSP_LED_SEG.c
* 说明    ：提供接口函数的源码
* 版 本 号：V1.0
* 修改记录：
*           版本号       日期         作者        
*	           V1.0     2022-10-3    yan xuewen        
*************************************************************/


/***************************************************************
                     本模块包含的头文件
***************************************************************/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "BSP_SEG.h"

#include "cmsis_os2.h"
#include "i2c.h"
uint8_t SEG_DATA[2]={0};
uint8_t const SEG_CODE[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};	

/*
参数说明
Seg1_Num：数码管1显示数字
Seg2_Num：数码管2显示数字
刷新率：约100HZ
*/


void BSP_LEDSEG_ShowNum(uint8_t Seg1_Num, uint8_t Seg2_Num)
{
	static int8_t display_pos=0;
	int8_t Temp_Data,i;
//1、更新显示数据
	SEG_DATA[0]=SEG_CODE[Seg1_Num%0x0f];
	if(Seg1_Num>0x0f)
		SEG_DATA[0]&=0x7f;
	
	
	SEG_DATA[1]=SEG_CODE[Seg2_Num%0x0f];	
	if(Seg2_Num>0x0f)
		SEG_DATA[1]&=0x7f;	

	
//2、关闭数码管阳极显示
	  HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin,GPIO_PIN_SET);

//3、显示数据
//3.1 选择显示的数码管	
	display_pos=(display_pos+1)%2;
	 switch (display_pos)
   {
    case 0:
     HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin,GPIO_PIN_RESET);//第1个数码管阳极置1
    break;
    case 1:
     HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin,GPIO_PIN_RESET);//第2个数码管阳极置1
    break;
		default:
		break;
	  }
//3.2 显示数码管段码
		i=8;
		Temp_Data=SEG_DATA[display_pos];
		do
		{
			HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_PIN,GPIO_PIN_RESET);			
			if((Temp_Data&0x80)==0)	  //向595输出一位，高位在前，低位在后
				HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_PIN,GPIO_PIN_RESET);			
			else
				HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_PIN,GPIO_PIN_SET);		
			Temp_Data=Temp_Data<<1;		  	//从165输入一位，低位在前，高位在后
			HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_PIN,GPIO_PIN_SET);					
			i--;
		}while (i!=0);
		
	HAL_GPIO_WritePin(RCK_GPIO_Port,RCK_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RCK_GPIO_Port,RCK_PIN,GPIO_PIN_SET);

}
