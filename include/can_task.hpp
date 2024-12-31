#ifndef CAN_TASK_HPP_
#define CAN_TASK_HPP_

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

void can_task_main(twai_message_t tx_message);
void printCanBusStatus(void);

#endif