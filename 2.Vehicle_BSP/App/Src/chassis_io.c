#include "chassis_io.h"
#include "board.h"
#include <string.h>

/* 上电后还没收到命令时，异步消息先走有线调试口。 */
static uint8_t reply_port = (uint8_t)SERIAL_DEBUG;

void ChassisIo_SetReplyPort(uint8_t port)
{
  reply_port = port;
}

void ChassisIo_Say(uint8_t port, const char *text)
{
  (void)Serial_Write((SerialPort)port, (const uint8_t *)text,
                     (uint16_t)strlen(text), 100U);
}

void ChassisIo_Reply(const char *text)
{
  ChassisIo_Say(reply_port, text);
}
