#include "control_system.h"	
/*电机  左A   右B*/
typedef enum {false = 0,true = 1} bool;
int L_coder, R_coder;          					//速度控制PID参数
int Motor_A, Motor_B;     
int OverflowTime=100;
volatile uint32_t millis = 0; // 记录毫秒数
volatile uint32_t seconds = 0; // 记录秒数
//电机PWM变量
float Target_MotorA, Target_MotorB;    						//电机目标值
float  Target_angle;                               //目标角度值
u8 	Car_mode = 0;														//模式变量
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PD控制
pwm+=KD[e（k）-e(k-1)]+Kp*e(k)
**************************************************************************/

int Incremental_PI_A(int Encoders_A, int Target_A)
{
    float Velocity_KP = 5.0, Velocity_KI = 0.002, Velocity_KD = 0.002;
    static int Pwm_A = 0;
    static int Integral_A = 0;
    static float Error_prev_A = 0;
    float MaxIntegral = 0.0;
    float MinIntegral = 0.0;
		bool isSaturation = false;
    float Error_A = (float)(Target_A - Encoders_A); // 计算偏差
    

    // 检查是否达到PWM输出上限或下限(主要针对Target_A为负值，且电机堵转的情况)
    if ((Pwm_A == 7199 && Error_A > 0) || (Pwm_A == -7199 && Error_A < 0))
    {
        isSaturation = true;
    }
    else
    {
        Integral_A += Error_A; // 积分项更新
        
        // 积分限幅
        MaxIntegral = (float)(7199 / Velocity_KI);
        MinIntegral = -(float)(7199 / Velocity_KI);
        if (Integral_A > MaxIntegral) Integral_A = MaxIntegral;
        else if (Integral_A < MinIntegral) Integral_A = MinIntegral;
    }

    Pwm_A += Velocity_KP * Error_A + Velocity_KD * (Error_A - Error_prev_A);
    
    if (!isSaturation)
    {
        Pwm_A += Velocity_KI * Integral_A;
			  printf("Incremental_PI_A :!isSaturation\r\n");
    }

    if (Pwm_A > 7199) Pwm_A = 7199;
    else if (Pwm_A < -7199) Pwm_A = -7199;

    Error_prev_A = Error_A; // 保存上一次偏差

    return Pwm_A; // 增量输出
}

int Incremental_PI_B(int Encoders_B, int Target_B)
{
		//参数与Incremental_PI_A中不一致，因电机的批次或者安装的影响，阻力不同，需要不同的P值
	  //电机阻力越大，P值相应的增大一些，保持两个电机几乎同时达到目标转速
	  //若追求效果可以调整I和D
    float Velocity_KP = 6.6, Velocity_KI = 0.002, Velocity_KD = 0.002;
    static int Pwm_B = 0;
    static int Integral_B = 0;
    static float Error_prev_B = 0;
    float MaxIntegral = 0.0;
    float MinIntegral = 0.0;
		bool isSaturation = false;
    float Error_B = (float)(Target_B - Encoders_B); // 计算偏差
    

    // 检查是否达到PWM输出上限或下限
    if ((Pwm_B == 7199 && Error_B > 0) || (Pwm_B == -7199 && Error_B < 0))
    {
        isSaturation = true;
    }
    else
    {
        Integral_B += Error_B; // 积分项更新
        
        // 积分限幅
        MaxIntegral = (float)(7199 / Velocity_KI);
        MinIntegral = -(float)(7199 / Velocity_KI);
        if (Integral_B > MaxIntegral) Integral_B = MaxIntegral;
        else if (Integral_B < MinIntegral) Integral_B = MinIntegral;
    }

    Pwm_B += Velocity_KP * Error_B + Velocity_KD * (Error_B - Error_prev_B);
    
    if (!isSaturation)
    {
        Pwm_B += Velocity_KI * Integral_B;
			  printf("Incremental_PI_B :!isSaturation\r\n");
    }

    if (Pwm_B > 7199) Pwm_B = 7199;
    else if (Pwm_B < -7199) Pwm_B = -7199;

    Error_prev_B = Error_B; // 保存上一次偏差

    return Pwm_B; // 增量输出
}


/**************************************************************************
函数功能：转每秒转脉冲数函数
入口参数：float
返回  值：int

电机ppr：700，倍频4
设定电机转速为1转/s，已知电机1转产生(700*4)脉冲，则每100ms产生的脉冲数为：(700*4)/(1000/100)，单位：脉冲数/100ms
**************************************************************************/
int Rs_To_CPR(float rads){  // rads取值范围：-1.5 ~ 1.5，即最大设定转速为1.5转/s
	int CRP=0;
	CRP=rads * ((700*4)/(1000/OverflowTime));
	return CRP;
}

/**************************************************************************
函数功能：控制电机的两轮转动速度
入口参数：float
返回  值：int
正交编码 (700*4*OverflowTime)/1000   1rad/s应该产生的的脉冲数
**************************************************************************/
void CalculateAndControlMotors(float RsA,float RsB){
	int TageA=0;
	int TageB=0;
	//读取OverflowTime ms时间的脉冲数
	L_coder=Read_Encoder(2);    
	R_coder=Read_Encoder(3);		
	printf("left  coder : %d\r\n",L_coder);
	printf("right coder : %d\r\n",R_coder);
	
	//计算OverflowTime时间转每秒的速度应达到的编码器值
	TageA=Rs_To_CPR(RsA);
	TageB=Rs_To_CPR(RsB);
	printf("TageA  coder : %d\r\n",TageA);
	printf("TageB  coder : %d\r\n",TageB);
	
	//===速度闭环控制计算电机B最终PWM 角度电机转动
	Motor_A=Incremental_PI_A(L_coder,TageA);                         //===速度闭环控制计算电机A最终PWM
	Motor_B=Incremental_PI_B(R_coder,TageB);
	
	printf("Motor_A  pwm : %d\r\n",Motor_A);
	printf("Motor_B  pwm : %d\r\n",Motor_B);
	
	
	Set_Pwm(Motor_A,Motor_B);      //设置电机转速
}


/**************************************************************************
函数功能：系统控制函数
入口参数：
返回  值：
**************************************************************************/
void System_Control(void)
{	
		/******获取解析数据帧********/	
		if(uart_rec_flag)                 //收到一帧数据
		{
			Target_MotorA=CAR_buff[1]/100.00; //左轮转速绝对值
			Target_MotorB=CAR_buff[3]/100.00; //右轮转速绝对值
			
			//转速值还原
			if(CAR_buff[0]==1){
				Target_MotorA=-1*Target_MotorA;
			}
			if(CAR_buff[2]==1){
				Target_MotorB=-1*Target_MotorB;
			}
			
			//倒车灯
			if(CAR_buff[0]==1 && CAR_buff[2]==1){
			   R_led_mode();
			}else{
				R_led_CLC();
			}
			
			uart_rec_flag=0;
			memset(CAR_buff,0,4);													//清除 等待获取下一帧
		}
		CalculateAndControlMotors(Target_MotorA,Target_MotorB);// 设置转速 pid介入
}


/**
  * @brief  系统滴答定时器中断服务函数
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
		millis++; // 每次滴答定时器中断，毫秒数加1
    if (millis % OverflowTime == 0) // 如果毫秒数达到100
    {
        millis = 0; // 毫秒数清零
        //seconds++; // 秒数加1
				
			System_Control();
    }
}





