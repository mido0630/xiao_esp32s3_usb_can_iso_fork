// arduino
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

// user code
#include "Debug_task_main.hpp"
#include "can_task.hpp"
#include "global_config.hpp"
#include "util_gptimer.hpp"

hw_timer_t *debug_gp_timer = NULL;

// RTOS hundle
TaskHandle_t tskHndl_can;
TaskHandle_t tskHndl_rs485;
TaskHandle_t tskHndl_ui;
TaskHandle_t tskHndl_debug;

twai_message_t tx_message;

void main_can(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_CAN_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while(true) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    tx_message.extd             = 1;                        // 拡張フレーム
    uint32_t base_id            = 0x31;                     // 基本となるID
    uint32_t sub_id             = 0x1234;                   // サブIDとしてのレスポンスID
    tx_message.identifier       = (base_id << 18) | sub_id; // 拡張ID
    tx_message.data_length_code = 8;                        // データ長8バイト
    tx_message.data[0]          = 0x00;                     // 送信データの設定
    tx_message.data[1]          = 0x01;
    tx_message.data[2]          = 0x02;
    tx_message.data[3]          = 0x03;
    tx_message.data[4]          = 0x04;
    tx_message.data[5]          = 0x05;
    tx_message.data[6]          = 0x06;
    tx_message.data[7]          = 0x07;
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::CAN_MAIN); // 処理時間計測開始
    can_task_main(tx_message);
    // printCanBusStatus();
    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::CAN_MAIN); // 処理時間計測停止
  }
}

void main_rs485(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_RS485_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while(true) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::RS485_MAIN); // 処理時間計測開始

    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::RS485_MAIN); // 処理時間計測停止
  }
}

void main_ui(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_UI_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while(true) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::UI_MAIN); // 処理時間計測開始

    digitalWrite(PIN::DEBUG_USER_LED, digitalRead(PIN::DEBUG_USER_LED) ^ 1);

    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::UI_MAIN); // 処理時間計測停止
  }
}

void setup() {
  // general purpose timer (8MHz count)
  debug_gp_timer = timerBegin(0, 10, true);
  init_debug_timer(debug_gp_timer);

  // USB uart
  Serial.begin(460800);                                                      // USB
  Serial1.begin(460800, SERIAL_8N1, PIN::DEBUG_UART_RX, PIN::DEBUG_UART_TX); // debug ext pin

  // PinConfig
  pinMode(PIN::DEBUG_USER_LED, OUTPUT);
  pinMode(PIN::PWR_COM_DEVICE, OUTPUT);
  pinMode(PIN::RS485_RE, OUTPUT);

  // CAN Config
  ESP32Can.begin(TWAI_SPEED_1000KBPS, PIN::CAN_TX, PIN::CAN_RX);

  // 電源投入後、一旦LOWに明示的に落とす
  digitalWrite(PIN::DEBUG_USER_LED, LOW);
  digitalWrite(PIN::PWR_COM_DEVICE, LOW);

  delay(300);

  // 外部デバイスの電源ON
  digitalWrite(PIN::PWR_COM_DEVICE, HIGH);

  // Task Config
  DEBUG::prepare_task();

  xTaskCreatePinnedToCore(main_can, "CAN", CAN_STACK_SIZE, NULL, 4, &tskHndl_can, APP_CPU_NUM);
  xTaskCreatePinnedToCore(main_rs485, "RS485", RS485_STACK_SIZE, NULL, 3, &tskHndl_rs485, APP_CPU_NUM);
  xTaskCreatePinnedToCore(main_ui, "UI", UI_STACK_SIZE, NULL, 2, &tskHndl_ui, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(DEBUG::main, "DEBUG", DEBUG_STACK_SIZE, NULL, 1, &tskHndl_debug, PRO_CPU_NUM);
}

void loop() {
}