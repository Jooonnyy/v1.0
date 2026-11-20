#ifndef VEHICLE_APP_H
#define VEHICLE_APP_H

/* 生产固件入口。调用前必须先完成 Board_InitStep6()，不能走 Board_Init()。 */
void VehicleApp_Begin(void);
void VehicleApp_Poll(void);

#endif
