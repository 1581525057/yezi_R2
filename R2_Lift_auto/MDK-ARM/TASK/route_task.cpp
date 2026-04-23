#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "omni_chassis.h"
#include "dji_motor.h"
#include "bsp_remove.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "PID.h"
#include "yun_j60.h"
#include "dm_imu.h"
#include "lift_auto.h"
#include "VescMotor.h"
#include "mieling.h"
#include "plan_route.h"

MeilingTarget_t text_posi;
uint16_t flag_mei = 0;
extern "C" void plan_route(void *argument)
{
	text_posi.preset_id = 0;
	text_posi.F_ref = 400.0f;
	text_posi.L_ref = 200.5f;
	text_posi.R_ref = 662.0f;
	text_posi.sensor_mask = SENSOR_ALL;
	text_posi.timeout_ms = 500000U;
	text_posi.tol_lat = 10.0f;
	text_posi.tol_lon = 10.0f;
	meiling.start(text_posi);
	for (;;)
	{
		if (flag_mei)
		{
			meiling.update();
		}

		osDelay(1);
	}
}
