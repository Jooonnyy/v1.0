#include "chassis_step_smoke.h"
#include "vehicle_app.h"

/* 逻辑已拆进 App（chassis_motion / chassis_task / vehicle_app）。
 * 这里只留台架入口，命令和时序与拆分前相同。 */
void ChassisStepSmoke_Begin(void)
{
  VehicleApp_Begin();
}

void ChassisStepSmoke_Poll(void)
{
  VehicleApp_Poll();
}
