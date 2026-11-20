#ifndef BOARD_INTERNAL_H
#define BOARD_INTERNAL_H
#include "board.h"
HAL_StatusTypeDef Board_GPIOInit(void);
HAL_StatusTypeDef Board_MotionInit(void);
HAL_StatusTypeDef Board_MotorPwmInit(void);
HAL_StatusTypeDef Board_CommInit(void);
void Board_SensorInit(void);
void Board_EncoderDEdge(void);
void Encoder_SampleCounts(void);
void Board_SerialIRQ(SerialPort port);
#endif
