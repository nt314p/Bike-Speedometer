#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include "hal_types.h"

void Speedometer_Init(uint8 task_id);
uint16 Speedometer_ProcessEvent(uint8 task_id, uint16 events);

#endif