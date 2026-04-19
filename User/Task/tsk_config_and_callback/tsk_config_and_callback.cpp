/**
 * @file tsk_config_and_callback.cpp
 * @author 
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 *
 */

#include "tsk_config_and_callback.h"

#include "main.h"
#include "drv_bsp.h"
#include "drv_can.h"
#include "motor_dm_task.h"
#include "motor_dji_task.h"

/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
static void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
  switch (Rx_Buffer->Header.StdId)
  {
    case (0x201):
    {
      DJI_M3508_CAN_RxCpltCallback(Rx_Buffer->Data);
      break;
    }
    case (0x00):
    {
      J4310_CAN_RxCpltCallback(Rx_Buffer->Data);
      break;
    }
    default:
    {
      break;
    }
  }
}

/**
 * @brief 全局任务初始化函数
 *
 * 调用位置：
 * - 在freeRtos.c中的MX_FREERTOS_Init()中被调用，作为FreeRTOS任务创建前的准备工作
 *
 * 功能：
 * - 初始化全局资源（例如CAN总线、GPIO等）
 */
void Task_Init(void)
{
	BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);
  	CAN_Init(&hcan1, CAN_Motor_Call_Back);

	// 电机任务函数的初始化
	DJI_M3508_Task_Init();
	J4310_Task_Init();
}