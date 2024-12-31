#include "debug_print.hpp"
#include "global_config.hpp"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#define CAN_DEBUG_LEVEL_SETTING DEBUG_WARN // デバッグレベル設定

twai_message_t received_message; // 受信メッセージ

void printCanBusStatus() {
  twai_status_info_t status;
  esp_err_t          result = twai_get_status_info(&status);

  if(result == ESP_OK) {
    Serial.println("---- CAN Bus Status ----");
    Serial.print("State: ");
    switch(status.state) {
    case TWAI_STATE_STOPPED:
      Serial.println("Stopped");
      break;
    case TWAI_STATE_RUNNING:
      Serial.println("Running");
      break;
    case TWAI_STATE_BUS_OFF:
      Serial.println("Bus Off");
      break;
    case TWAI_STATE_RECOVERING:
      Serial.println("Recovering");
      break;
    default:
      Serial.println("Unknown");
      break;
    }

    Serial.print("Messages to TX: ");
    Serial.println(status.msgs_to_tx);

    Serial.print("Messages to RX: ");
    Serial.println(status.msgs_to_rx);

    Serial.print("TX Error Counter: ");
    Serial.println(status.tx_error_counter);

    Serial.print("RX Error Counter: ");
    Serial.println(status.rx_error_counter);

    Serial.print("TX Failed Count: ");
    Serial.println(status.tx_failed_count);

    Serial.print("RX Missed Count: ");
    Serial.println(status.rx_missed_count);

    Serial.println("-------------------------");
  } else {
    Serial.println("Failed to get CAN bus status.");
  }
}

void handleStopError() {
  twai_status_info_t status;

  if(twai_get_status_info(&status) == ESP_OK) { // 現在の状態を取得
    debugPrint(DEBUG_INFO, "Current TWAI State: %d", status.state);

    if(status.state == TWAI_STATE_RUNNING) { // RUNNING 状態の場合は停止
      if(twai_stop() == ESP_OK) {
        debugPrint(DEBUG_INFO, "TWAI driver stopped successfully.");
        return;
      } else {
        debugPrint(DEBUG_ERROR, "Failed to stop TWAI driver.");
      }
    }
    if(status.state == TWAI_STATE_BUS_OFF) { // BUS OFF 状態の場合はリカバリ
      debugPrint(DEBUG_WARN, "Driver is in BUS OFF state. Attempting recovery...");
      if(twai_initiate_recovery() == ESP_OK) {
        debugPrint(DEBUG_INFO, "Recovery process started successfully.");
        return;
      } else {
        debugPrint(DEBUG_ERROR, "Failed to initiate recovery.");
      }
    }
  } else {
    debugPrint(DEBUG_ERROR, "Failed to retrieve TWAI driver status.");
  }
  // ドライバのアンインストールと再インストール
  debugPrint(DEBUG_WARN, "Attempting to reinstall TWAI driver...");
  if(twai_driver_uninstall() == ESP_OK) {
    debugPrint(DEBUG_INFO, "TWAI driver uninstalled successfully.");
  } else {
    debugPrint(DEBUG_ERROR, "Failed to uninstall TWAI driver.");
  }
  // 再設定
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(PIN::CAN_TX),
      static_cast<gpio_num_t>(PIN::CAN_RX),
      TWAI_MODE_NORMAL);
  // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_6, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    debugPrint(DEBUG_INFO, "TWAI driver reinstalled successfully.");
    if(twai_start() == ESP_OK) {
      debugPrint(DEBUG_INFO, "TWAI driver restarted successfully.");
    } else {
      debugPrint(DEBUG_ERROR, "Failed to restart TWAI driver.");
    }
  } else {
    debugPrint(DEBUG_ERROR, "Failed to reinstall TWAI driver.");
  }
}

// エラー時の処理関数
void handleTransmitError(uint32_t base_id, const twai_message_t &tx_message) {
  twai_status_info_t status;
  esp_err_t          result = twai_get_status_info(&status);

  if(result == ESP_OK) {
    // 状態情報をデバッグ出力
    debugPrint(DEBUG_ERROR, "CAN Transmit Error - Status Info:");
    debugPrint(DEBUG_ERROR, "Base ID: 0x%X", base_id);
    debugPrint(DEBUG_ERROR, "State: %s",
               (status.state == TWAI_STATE_STOPPED ? "Stopped" : (status.state == TWAI_STATE_RUNNING ? "Running" : (status.state == TWAI_STATE_BUS_OFF ? "Bus Off" : (status.state == TWAI_STATE_RECOVERING ? "Recovering" : "Unknown")))));
    debugPrint(DEBUG_ERROR, "Messages to TX: %d", status.msgs_to_tx);
    debugPrint(DEBUG_ERROR, "Messages to RX: %d", status.msgs_to_rx);
    debugPrint(DEBUG_ERROR, "TX Error Counter: %d", status.tx_error_counter);
    debugPrint(DEBUG_ERROR, "RX Error Counter: %d", status.rx_error_counter);
    debugPrint(DEBUG_ERROR, "TX Failed Count: %d", status.tx_failed_count);
    debugPrint(DEBUG_ERROR, "RX Missed Count: %d", status.rx_missed_count);
  } else {
    debugPrint(DEBUG_ERROR, "Failed to retrieve status info.");
  }

  // Bus Off 状態の場合、復帰処理を実施
  if(status.state == TWAI_STATE_BUS_OFF) {
    debugPrint(DEBUG_INFO, "TWAI driver in Bus Off state. Initiating recovery...");
    if(twai_initiate_recovery() != ESP_OK) {
      debugPrint(DEBUG_ERROR, "Failed to initiate TWAI recovery.");
      return;
    }
  }

  // ドライバの停止
  debugPrint(DEBUG_INFO, "Stopping TWAI driver...");
  if(twai_stop() != ESP_OK) {
    debugPrint(DEBUG_ERROR, "Failed to stop TWAI driver.");
    handleStopError();
    return;
  }

  // 送信キューのクリア
  debugPrint(DEBUG_INFO, "Clearing TX queue...");
  if(twai_clear_transmit_queue() != ESP_OK) {
    debugPrint(DEBUG_ERROR, "Failed to clear TX queue.");
    return;
  }

  // 受信キューのクリア
  debugPrint(DEBUG_INFO, "Clearing RX queue...");
  if(twai_clear_receive_queue() != ESP_OK) {
    debugPrint(DEBUG_ERROR, "Failed to clear RX queue.");
    return;
  }

  // ドライバの再起動
  debugPrint(DEBUG_INFO, "Restarting TWAI driver...");
  if(twai_start() != ESP_OK) {
    debugPrint(DEBUG_ERROR, "Failed to restart TWAI driver.");
    return;
  }

  // 再起動後の状態確認
  result = twai_get_status_info(&status);
  if(result == ESP_OK && status.state == TWAI_STATE_RUNNING) {
    debugPrint(DEBUG_INFO, "TWAI driver restarted successfully.");
  } else {
    debugPrint(DEBUG_ERROR, "TWAI driver failed to restart.");
  }
}

void processResSummary2(const twai_message_t &message) {
  uint32_t base_id = (message.identifier >> 18) & 0x7FF; // base_id を抽出
  uint32_t cmd_id  = message.identifier & 0x3FFFF;       // CMD ID (sub_id) を抽出

  if(cmd_id != 0x1234) { // CMD ID が期待通りか確認
    debugPrint(DEBUG_WARN, "Unexpected CMD ID received. Base ID: 0x%X, CMD ID: 0x%X", base_id, cmd_id);
    return;
  }

  float joint_angle   = (float)(int16_t)((message.data[1] << 8) | message.data[0]) / 32768.0f * 180.0f; // [deg]
  float motor_current = (float)(int16_t)((message.data[3] << 8) | message.data[2]) / 1000.0f;           // [A]
  float motor_voltage = (float)(int16_t)((message.data[5] << 8) | message.data[4]) / 1000.0f;           // [V]
  float vm_voltage    = (float)(int16_t)((message.data[7] << 8) | message.data[6]) / 1000.0f;           // [V]

  debugPrint(DEBUG_DETAIL, "Base ID: 0x%X, Joint Angle: %.2f deg, Motor Current: %.2f A, Motor Voltage: %.2f V, VM Voltage: %.2f V",
             base_id, joint_angle, motor_current, motor_voltage, vm_voltage);
}

void can_task_main(twai_message_t tx_message) {
  setDebugLevel(CAN_DEBUG_LEVEL_SETTING);
  // 送信
  if(twai_transmit(&tx_message, pdMS_TO_TICKS(1)) == ESP_OK) {
    debugPrint(DEBUG_INFO, "CAN message sent");
  } else {
    debugPrint(DEBUG_ERROR, "Failed to send CAN message");
    uint32_t base_id = (tx_message.identifier >> 18) & 0x7FF;
    // handleTransmitError(base_id, tx_message);
    debugPrint(DEBUG_INFO, "Clearing TX queue...");
    if(twai_clear_transmit_queue() != ESP_OK) {
      debugPrint(DEBUG_ERROR, "Failed to clear TX queue.");
      return;
    }
    return;
  }

  // 受信
  twai_message_t rx_message;
  if(twai_receive(&rx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
    received_message = rx_message;
    processResSummary2(received_message);
  } else {
    debugPrint(DEBUG_WARN, "No CAN message received");
  }
}