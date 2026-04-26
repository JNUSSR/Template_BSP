/**
 * @file uart_wheelchassis.h
 * @author 
 * @brief 底盘电控串口通信接口 + 斜坡规划
 *        接收上位机VX, VY, VW控制指令，进行斜坡规划后下发到底盘控制周期函数
 * @version 0.1
 * @date 2026-4-22 0.1 26赛季定稿
 *
 * @copyright 
 *
 */
#include "uart_wheelchassis.h"

/**
 * @brief 底盘初始化函数
 *
 * @param hcan CAN句柄指针，允许外部指定用于底盘控制的CAN总线
 */
void Class_Uart_Wheelchassis::Init(CAN_HandleTypeDef* hcan)
{
	// 先复用底层底盘初始化，保持与crt一致的PID与基础配置。
	Class_Chassis_Omni::Init();

	hcan_ = hcan;

	// 允许外部指定CAN句柄（与遥控版本一致），用于不同总线切换。
	if (hcan_ != nullptr)
	{
		for (int i = 0; i < 4; i++)
		{
			Chassis_Motor[i].Init(hcan_, Chassis_Motor_Id_Enum[i], Control_Method_OMEGA, 19.0f, 20.0f);
		}
	}

	InitSlopePlanner();
	SetMotion(0.0f, 0.0f, 0.0f);
}

/**
 * @brief 设置底盘运动状态
 *
 * @param vx 目标线速度X分量
 * @param vy 目标线速度Y分量
 * @param vw 目标角速度Z分量
 */
void Class_Uart_Wheelchassis::SetMotion(float vx, float vy, float vw)
{
	target_vx_raw_ = ApplyDeadzone(vx, SPEED_DEADZONE);
	target_vy_raw_ = ApplyDeadzone(vy, SPEED_DEADZONE);
	target_vw_raw_ = ApplyDeadzone(vw, SPEED_DEADZONE);
}

/**
 * @brief 带斜坡规划的底盘控制函数
 *
 * @param vx 目标线速度X分量
 * @param vy 目标线速度Y分量
 * @param vw 目标角速度Z分量
 */
void Class_Uart_Wheelchassis::WithSlopeTask(float vx, float vy, float vw)
{
	SetMotion(vx, vy, vw);
	WithSlopeTask();
}

/**
 * @brief 斜坡规划控制函数
 *
 * 功能：
 *	- 将当前目标速度设置给斜坡规划器，让其根据当前输出和目标自动计算斜坡过渡
 *	- 将斜坡规划器的输出作为新的目标速度，更新到底盘控制周期函数中
 *	- 调用底盘控制周期函数，进行PID计算并下发控制量
 */
void Class_Uart_Wheelchassis::WithSlopeTask()
{
	// 使用上次输出作为当前真实值，避免规划器重新从0启动。
	slope_vx_.Set_Now_Real(slope_vx_.Get_Out());
	slope_vy_.Set_Now_Real(slope_vy_.Get_Out());
	slope_vw_.Set_Now_Real(slope_vw_.Get_Out());

	slope_vx_.Set_Target(target_vx_raw_);
	slope_vy_.Set_Target(target_vy_raw_);
	slope_vw_.Set_Target(target_vw_raw_);

	slope_vx_.TIM_Calculate_PeriodElapsedCallback();
	slope_vy_.TIM_Calculate_PeriodElapsedCallback();
	slope_vw_.TIM_Calculate_PeriodElapsedCallback();

	Set_Target_Velocity_X(slope_vx_.Get_Out());
	Set_Target_Velocity_Y(slope_vy_.Get_Out());
	Set_Target_Omega(slope_vw_.Get_Out());

	TIM_1ms_Control_PeriodElapsedCallback();
}

/**
 * @brief 设置斜坡规划步长
 *
 * @param v_inc 线速度增加步长
 * @param v_dec 线速度减少步长
 * @param w_inc 角速度增加步长
 * @param w_dec 角速度减少步长
 */
void Class_Uart_Wheelchassis::SetSlopeStep(float v_inc, float v_dec, float w_inc, float w_dec)
{
	slope_vx_.Set_Increase_Value(v_inc);
	slope_vx_.Set_Decrease_Value(v_dec);

	slope_vy_.Set_Increase_Value(v_inc);
	slope_vy_.Set_Decrease_Value(v_dec);

	slope_vw_.Set_Increase_Value(w_inc);
	slope_vw_.Set_Decrease_Value(w_dec);
}

/**
 * @brief 初始化斜坡规划器
 */
void Class_Uart_Wheelchassis::InitSlopePlanner()
{
	slope_vx_.Init(VELOCITY_INCREASE_STEP, VELOCITY_DECREASE_STEP, Slope_First_REAL);
	slope_vy_.Init(VELOCITY_INCREASE_STEP, VELOCITY_DECREASE_STEP, Slope_First_REAL);
	slope_vw_.Init(ANGULAR_INCREASE_STEP, ANGULAR_DECREASE_STEP, Slope_First_REAL);

	slope_vx_.Set_Now_Real(0.0f);
	slope_vy_.Set_Now_Real(0.0f);
	slope_vw_.Set_Now_Real(0.0f);
	slope_vx_.Set_Target(0.0f);
	slope_vy_.Set_Target(0.0f);
	slope_vw_.Set_Target(0.0f);
}

/**
 * @brief 应用死区，避免小输入导致底盘抖动
 *
 * @param value 输入值
 * @param deadzone 死区范围，绝对值小于该范围的输入将被视为0
 * @return float 处理后的输出值
 */
float Class_Uart_Wheelchassis::ApplyDeadzone(float value, float deadzone)
{
	if (value > -deadzone && value < deadzone)
	{
		return 0.0f;
	}
	return value;
}



