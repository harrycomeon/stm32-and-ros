/*********************************************************************************************
程序名：	USART串口控制程序
改编自：洋桃电子
编写人：	harry	
编写时间：	2018年6月8日
硬件支持：	STM32F103RCT6  外部晶振8MHz RCC函数设置主频72MHz　  


							
说明：
 # 可根据自己的需要增加或删减。

*********************************************************************************************/
#include "stm32f10x.h" //STM32头文件
#include "sys.h"
#include "led.h"   //包含对IO工作方式初始化
#include "usart.h"


int main (void){//主程序
	u8 a;
	//初始化程序
	RCC_Configuration(); //时钟设置
	LED_Init();//LED初始化
	USART1_Init(115200); //串口初始化（参数是波特率）

	//主循环
	while(1){

		//查询方式接收
		if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) != RESET){  //查询串口待处理标志位
			a =USART_ReceiveData(USART1);//读取接收到的数据
			switch (a){
				case '0':
					GPIO_WriteBit(GPIOB,GPIO_Pin_0,(BitAction)(0)); //控制PB0口，作为信号输入端，写入0
					printf("%c:PB0 OFF ",a); //
					break;
				case '1':
					GPIO_WriteBit(GPIOB,GPIO_Pin_0,(BitAction)(1)); //控制PB0口，作为信号输入端，写入1
					printf("%c:PB0 ON ",a); //
					break;
				default:
					break;
			}		  
		}
	}
}




