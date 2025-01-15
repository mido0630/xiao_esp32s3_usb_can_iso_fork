#ifndef CAN_TASK_HPP_
#define CAN_TASK_HPP_

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

#define CAN_DATA_LENGTH 8

void can_task_main(void);
void printCanBusStatus(void);

#endif