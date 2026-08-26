#include "stm32f10x.h"
#include "sys.h"
#include "control_system.h"
int main(void)
  { 
		RCC->CSR|=1<<24;
		Stm32_Clock_Init(9);						//�ⲿʱ��8Mhz 9��Ƶ  8*9= 72mhz��Ƶ72mhz
		MY_NVIC_PriorityGroupConfig(2);	//=====�ж����ȼ�����		
		uart_init(115200);	            //=====���ڳ�ʼ��Ϊ115200
		JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
		JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���

		colorful_led_Init();            //=====�ŲʵƳ�ʼ��
        PWM_Init(7199, 9);
		printf("QST����\r\n");
		/**��Ҫ����**/
	while(1)
	{
		Set_Pwm(2500,2500);
		delay_ms(100);
	}
}
	

