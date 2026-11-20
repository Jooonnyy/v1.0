#ifndef CHASSIS_IO_H
#define CHASSIS_IO_H
#include <stdint.h>

/* 命令回复口。运动层和任务层只负责把原来那句英文发出去，不自己选串口。
 * 调用前由命令解析把回复口设成"这条命令是从哪口来的"。 */
void ChassisIo_SetReplyPort(uint8_t port);
void ChassisIo_Say(uint8_t port, const char *text);
void ChassisIo_Reply(const char *text);

#endif
