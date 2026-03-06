#include "motor.h"
/**
************************************************************************************************
* @brief    电机控制任务，运动算法
* @param    None
* @return   None
* @author   创源启明		2025.10.28
************************************************************************************************
**/
// 外部变量声明
WINGS_DATA Wings_Data;

/************ 运动速度变量定义 *************/

int motor_1_set_pwm;
int motor_2_set_pwm;
int motor_3_set_pwm;
int motor_4_set_pwm;

/************ PID结构体变量定义 *************/
pid_type_def motor_1_pid,motor_2_pid,motor_3_pid,motor_4_pid;
/************** 速度PID限幅设置 *************/
const float motore_max_out=17999,motor_max_iout=4095; //速度环总输出限幅 积分项限幅

/**电机PID初始化**/
void Chassis_PID_Init(void) 
{
		const float motor_1_speed_pid[3] = {KP1, KI1, KD1};
		const float motor_3_speed_pid[3] = {KP3, KI3, KD3};

    const float motor_2_speed_pid[3] = {KP2, KI2, KD2};
    const float motor_4_speed_pid[3] = {KP4, KI4, KD4};

	  PID_init(&motor_1_pid, PID_POSITION, motor_1_speed_pid, motore_max_out, motor_max_iout);
	  PID_init(&motor_3_pid, PID_POSITION, motor_3_speed_pid, motore_max_out, motor_max_iout);

    PID_init(&motor_2_pid, PID_POSITION, motor_2_speed_pid, motore_max_out, motor_max_iout);
    PID_init(&motor_4_pid, PID_POSITION, motor_4_speed_pid, motore_max_out, motor_max_iout);
		
}


/*************** 电机PID计算 ****************/
void Motor_PID_Control(void)
{
	
	Wings_Data.Wings_motor[0].Target_Speed=motor_1_set_pwm=PID_calc(&motor_1_pid, Wings_Data.Wings_motor[0].Corrective_Angle , Wings_Data.Wings_motor[0].Target_Angle );
	Wings_Data.Wings_motor[2].Target_Speed=motor_3_set_pwm=-PID_calc(&motor_3_pid, Wings_Data.Wings_motor[2].Corrective_Angle , Wings_Data.Wings_motor[2].Target_Angle );

    Wings_Data.Wings_motor[1].Target_Speed=motor_2_set_pwm=PID_calc(&motor_2_pid, Wings_Data.Wings_motor[1].Corrective_Angle , Wings_Data.Wings_motor[1].Target_Angle );
    Wings_Data.Wings_motor[3].Target_Speed=motor_4_set_pwm=-PID_calc(&motor_4_pid, Wings_Data.Wings_motor[3].Corrective_Angle , Wings_Data.Wings_motor[3].Target_Angle );


//这个函数的电机对应（3，0，1，2）
//	Set_Pwm(0,-(Wings_Data.Wings_motor[0].Target_Speed),0,0);
//		Set_Pwm(0,0,Wings_Data.Wings_motor[1].Target_Speed,0);
//	Set_Pwm(0,0,0,Wings_Data.Wings_motor[2].Target_Speed);
//		Set_Pwm(Wings_Data.Wings_motor[3].Target_Speed,0,0,0);
		Set_Pwm(Wings_Data.Wings_motor[3].Target_Speed,-(Wings_Data.Wings_motor[0].Target_Speed),Wings_Data.Wings_motor[1].Target_Speed,Wings_Data.Wings_motor[2].Target_Speed);
}




/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : motor1_out - 电机1输出值, motor2_out - 电机2输出值
          motor3_out - 电机3输出值, motor4_out - 电机4输出值
Output  : none
函数功能：赋值给PWM寄存器，控制四个车轮转速与方向及舵机角度
入口参数：motor1_out - 电机1的输出值（来自Wings_Data或PID计算）
          motor2_out - 电机2的输出值
          motor3_out - 电机3的输出值  
          motor4_out - 电机4的输出值
返回  值：无
**************************************************************************/



// 分支消除的 int16 绝对值（两补码）
static inline uint16_t abs16_fast(int16_t x) {
    int16_t m = x >> 15;            // x<0 → -1；x>=0 → 0
    return (uint16_t)((x ^ m) - m); // 等价于 abs(x)
}

void Set_Pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    // -------- Motor 1 --------
    uint16_t pwm1  = abs16_fast(m3);        //eg 1000 ---> 1000,0  -1000---> 0,1000
    uint16_t mask1 = (uint16_t)-(m3 > 0);      // >0 → 0xFFFF；<=0 → 0x0000
    PWM_M1_2 = (uint16_t)(pwm1 & mask1);       // 正转：CH2=pwm
    PWM_M1_1 = (uint16_t)(pwm1 & ~mask1);      // 反/零：CH1=pwm


    // -------- Motor 2 --------
    uint16_t pwm2  = abs16_fast(m2);
    uint16_t mask2 = (uint16_t)-(m2 > 0);      // >0 → 0xFFFF；<=0 → 0x0000
    PWM_M2_2 = (uint16_t)(pwm2 & mask2);       // 正转：CH2=pwm
    PWM_M2_1 = (uint16_t)(pwm2 & ~mask2);      // 反/零：CH1=pwm

    // -------- Motor 3 --------
    uint16_t pwm3  = abs16_fast(m1);
    uint16_t mask3 = (uint16_t)-(m1 > 0);
    PWM_M3_2 = (uint16_t)(pwm3 & mask3);
    PWM_M3_1 = (uint16_t)(pwm3 & ~mask3);

    // -------- Motor 4 --------
    uint16_t pwm4  = abs16_fast(m4);
    uint16_t mask4 = (uint16_t)-(m4 > 0);      // >0 → 0xFFFF；<=0 → 0x0000
    PWM_M4_2 = (uint16_t)(pwm4 & mask4);       // 正转：CH2=pwm
    PWM_M4_1 = (uint16_t)(pwm4 & ~mask4);      // 反/零：CH1=pwm
}
