#include "route_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "omni_chassis.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "lift_auto.h"
#include "mieling.h"
#include "plan_route.h"
#include "usart_task.h"

// x:0.90 y :-0.33 yaw:0.0
// dt35 left :361.5  forward:148.0

ROUTE_TASK route_t;

namespace
{
	/* ---- 调试用坐标点，仅在本文件内有效 ---- */

	/* 第一次重定位目标点 */
	MeilingTarget_t first_relocation = {
		.preset_id = 0,
		.L_ref = 361.5f,	   /* 左侧传感器参考值（mm） */
		.R_ref = 662.0f,	   /* 右侧传感器参考值（mm） */
		.F_ref = 148.0f,	   /* 前方传感器参考值（mm） */
		.tol_lat = 6.0f,	   /* 横向容差（mm） */
		.tol_lon = 6.0f,	   /* 纵向容差（mm） */
		.timeout_ms = 500000U, /* 超时时间（ms） */
		.sensor_mask = SENSOR_FRONT | SENSOR_LEFT,
	};

	/* 第二次重定位目标点 */
	MeilingTarget_t second_relocation = {
		.preset_id = 1,
		.L_ref = 200.5f,	   /* 左侧传感器参考值（mm） */
		.R_ref = 662.0f,	   /* 右侧传感器参考值（mm） */
		.F_ref = 400.0f,	   /* 前方传感器参考值（mm） */
		.tol_lat = 10.0f,	   /* 横向容差（mm） */
		.tol_lon = 10.0f,	   /* 纵向容差（mm） */
		.timeout_ms = 500000U, /* 超时时间（ms） */
		.sensor_mask = SENSOR_ALL,
	};
}

void ROUTE_TASK::route_reset()
{
	state = PHASE_IDLE;
	flag = 0;
	flag_relocation = 0;
	relocation_number = 0;
}

void ROUTE_TASK::meiling_route()
{
	if (flag != 1)
		return;

	if (state == PHASE_IDLE)
		state = FIRST_RELOCATION;

	switch (state)
	{
	case FIRST_RELOCATION:
		if (flag_relocation)
		{
			if (relocation_number == 0)
			{
				meiling.start(first_relocation);
			}
			flag_relocation = 0;
			relocation_number++;
		}

		break;

	default:
		break;
	}
}
extern "C" void plan_route(void *argument)
{

	for (;;)
	{

		osDelay(1);
	}
}
