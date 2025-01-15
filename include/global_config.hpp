#ifndef GLOBAL_CONFIG_HPP_
#define GLOBAL_CONFIG_HPP_

#include <Arduino.h>

/************************ PinAssign設定 ここから ************************/
namespace PIN {

static constexpr uint8_t DEBUG_USER_LED = LED_BUILTIN;
static constexpr uint8_t DEBUG_UART_TX  = D0;
static constexpr uint8_t DEBUG_UART_RX  = D1;

static constexpr uint8_t PWR_COM_DEVICE = D3;

static constexpr uint8_t CAN_TX = D4;
static constexpr uint8_t CAN_RX = D5;

static constexpr uint8_t RS485_RE = D8;
static constexpr uint8_t RS485_RX = D9;
static constexpr uint8_t RS485_TX = D10;

}; // namespace PIN
/************************ PinAssign設定 ここまで ************************/

/************************ RTOS設定 ここから ************************/
#define CAN_STACK_SIZE (4096)
#define RS485_STACK_SIZE (4096)
#define UI_STACK_SIZE (4096)
#define DEBUG_STACK_SIZE (4096)

static constexpr uint32_t LOOP_RATE_CAN_HZ   = 100;
static constexpr uint32_t LOOP_RATE_RS485_HZ = 200;
static constexpr uint32_t LOOP_RATE_UI_HZ    = 100;
static constexpr uint32_t LOOP_RATE_DEBUG_HZ = 100;

/************************ RTOS設定 ここまで ************************/

/************************ DEBUG PRINT設定 ここから ************************/
#define DEBUG_SERIAL_MOD (Serial)
// #define DEBUG_SERIAL_MOD (Serial1)

#include "Debug_task_main.hpp"
template <typename... Args>
void debug_printf(const char *format, Args const &...args) {
  uint16_t u16_print_size = sprintf((char *)DEBUG::EXT_PRINT_BUF, format, args...);
  if(u16_print_size >= 1024) u16_print_size = 1024;
  DEBUG::print(DEBUG::EXT_PRINT_BUF, u16_print_size);
}

// #define DEBUG_PRINT_CAN(fmt, ...) debug_printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_CAN(fmt, ...)

// #define DEBUG_PRINT_STR_CAN(fmt) debug_printf(fmt)
#define DEBUG_PRINT_STR_CAN(fmt)

/************************ DEBUG PRINT設定 ここまで ************************/

/************************ DEBUG TASK負荷測定設定 ここから ************************/
#define ENABLE_PRINT_PROCESS_LOAD (1)

enum DBG_PRC_ID {
  CAN_MAIN   = 0x10,
  RS485_MAIN = 0x20,
  UI_MAIN    = 0x30,
  DBG_MAIN   = 0xF0,
};

#if ENABLE_PRINT_PROCESS_LOAD
#define DEBUG_PRINT_PRC_START(proc_id) DEBUG::record_proc_load(proc_id, 1)
#define DEBUG_PRINT_PRC_FINISH(proc_id) DEBUG::record_proc_load(proc_id, 0)
#else
#define DEBUG_PRINT_PRC_START(proc_id)
#define DEBUG_PRINT_PRC_FINISH(proc_id)
#endif

/************************ DEBUG TASK負荷測定設定 ここまで ************************/

#endif