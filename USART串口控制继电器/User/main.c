/*********************************************************************************************
��������	USART���ڿ��Ƴ���
�ı��ԣ����ҵ���
��д�ˣ�	harry	
��дʱ�䣺	2018��6��8��
Ӳ��֧�֣�	STM32F103RCT6  �ⲿ����8MHz RCC����������Ƶ72MHz��  


							
˵����
 # �ɸ����Լ�����Ҫ���ӻ�ɾ����

*********************************************************************************************/
#include "stm32f10x.h" //STM32ͷ�ļ�
#include "sys.h"
#include "led.h"   //������IO������ʽ��ʼ��
#include "usart.h"


int main (void){//������
	u8 a;
	//��ʼ������
	RCC_Configuration(); //ʱ������
	LED_Init();//LED��ʼ��
	USART1_Init(115200); //���ڳ�ʼ���������ǲ����ʣ�

	//��ѭ��
	while(1){

		//��ѯ��ʽ����
		if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) != RESET){  //��ѯ���ڴ������־λ
			a =USART_ReceiveData(USART1);//��ȡ���յ�������
			switch (a){
				case '0':
					GPIO_WriteBit(GPIOB,GPIO_Pin_0,(BitAction)(0)); //����PB0�ڣ���Ϊ�ź�����ˣ�д��0
					printf("%c:PB0 OFF ",a); //
					break;
				case '1':
					GPIO_WriteBit(GPIOB,GPIO_Pin_0,(BitAction)(1)); //����PB0�ڣ���Ϊ�ź�����ˣ�д��1
					printf("%c:PB0 ON ",a); //
					break;
				default:
					break;
			}		  
		}
	}
}




