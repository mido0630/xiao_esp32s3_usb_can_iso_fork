#include "can_task.hpp"
#include "debug_print.hpp"
#include "global_config.hpp"
#include "joint_data_control.hpp"
#include <ESP32-TWAI-CAN.hpp>
#define CAN_DEBUG_LEVEL_SETTING DEBUG_WARN // デバッグレベル設定
twai_message_t tx_msg;
twai_message_t rx_msg;

void printCanBusStatus() {
  twai_status_info_t status;
  esp_err_t          result = twai_get_status_info(&status);

  if(result == ESP_OK) {
    Serial.print("[CAN]\t");
    Serial.print("St: "); // State
    switch(status.state) {
    case TWAI_STATE_STOPPED:
      Serial.print("Stop");
      break;
    case TWAI_STATE_RUNNING:
      Serial.print("Run");
      break;
    case TWAI_STATE_BUS_OFF:
      Serial.print("Off");
      break;
    case TWAI_STATE_RECOVERING:
      Serial.print("Rec");
      break;
    default:
      Serial.print("Unk");
      break;
    }
    Serial.print("\tTXQ: "); // Messages to TX
    Serial.print(status.msgs_to_tx);
    Serial.print("\tRXQ: "); // Messages to RX
    Serial.print(status.msgs_to_rx);
    Serial.print("\tTXE: "); // TX Error Counter
    Serial.print(status.tx_error_counter);
    Serial.print("\tRXE: "); // RX Error Counter
    Serial.print(status.rx_error_counter);
    Serial.print("\tTXF: "); // TX Failed Count
    Serial.print(status.tx_failed_count);
    Serial.print("\tRXM: "); // RX Missed Count
    Serial.println(status.rx_missed_count);
  } else {
    Serial.println("[CAN]\tStatus Failed");
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
void handleTransmitError(uint32_t base_id, const twai_message_t &tx_msg) {
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

void processResSummary(const twai_message_t &message) {
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
  // joint_state_list から base_id に対応するエントリを探す
  for(auto &joint : joint_state_list) {
    if(joint.joint_id == base_id) {                            // 対応する関節を発見
      joint.angle         = joint_angle;                       // 関節角度を更新
      joint.current       = motor_current;                     // モーター電流を更新
      joint.motor_voltage = motor_voltage;                     // モーター電圧を更新
      joint.vm_voltage    = static_cast<uint32_t>(vm_voltage); // VM電圧を更新
      debugPrint(DEBUG_INFO, "Updated joint state for Base ID: 0x%X", base_id);
      break;
    }
  }
}

void make_tx_header(uint8_t joint_id, cmd_id command, cmd_id flag = cmd_id::WITH_RES) {
  tx_msg.identifier = (static_cast<uint32_t>(joint_id) << 18) |
                      (static_cast<uint16_t>(command) | static_cast<uint16_t>(flag));
  tx_msg.extd             = 1;
  tx_msg.data_length_code = CAN_DATA_LENGTH;
}
void make_tx_data_fill_zero() {
  for(uint8_t i = 0; i < CAN_DATA_LENGTH; ++i) {
    tx_msg.data[i] = 0x00;
  }
}
void make_tx_data_move_angle(float target_angle, uint32_t move_time, float current_limit) {
  // target_angle s15.16 [deg] LE 形式
  int32_t angle_s15_16 = static_cast<int32_t>(target_angle * 32768.0f); // 2^15
  tx_msg.data[0]       = static_cast<uint8_t>(angle_s15_16 & 0xFF);
  tx_msg.data[1]       = static_cast<uint8_t>((angle_s15_16 >> 8) & 0xFF);
  tx_msg.data[2]       = static_cast<uint8_t>((angle_s15_16 >> 16) & 0xFF);
  tx_msg.data[3]       = static_cast<uint8_t>((angle_s15_16 >> 24) & 0xFF);
  // move_time  [ms] LE 形式
  tx_msg.data[4] = static_cast<uint8_t>(move_time & 0xFF);
  tx_msg.data[5] = static_cast<uint8_t>((move_time >> 8) & 0xFF);
  // current_limit s8.8 [A] LE 形式
  uint16_t current_s8_8 = static_cast<uint16_t>(current_limit * 256.0f);
  tx_msg.data[6]        = static_cast<uint8_t>(current_s8_8 & 0xFF);
  tx_msg.data[7]        = static_cast<uint8_t>((current_s8_8 >> 8) & 0xFF);
}

void make_tx_data_set_target_current(float target_iq, float target_id) {
  // target_iq s15.16 [A] LE 形式
  int32_t iq_s15_16 = static_cast<int32_t>(target_iq * 32768.0f);
  tx_msg.data[0]    = static_cast<uint8_t>(iq_s15_16 & 0xFF);
  tx_msg.data[1]    = static_cast<uint8_t>((iq_s15_16 >> 8) & 0xFF);
  tx_msg.data[2]    = static_cast<uint8_t>((iq_s15_16 >> 16) & 0xFF);
  tx_msg.data[3]    = static_cast<uint8_t>((iq_s15_16 >> 24) & 0xFF);
  // target_id s15.16 [A] LE 形式
  int32_t id_s15_16 = static_cast<int32_t>(target_id * 32768.0f);
  tx_msg.data[4]    = static_cast<uint8_t>(id_s15_16 & 0xFF);
  tx_msg.data[5]    = static_cast<uint8_t>((id_s15_16 >> 8) & 0xFF);
  tx_msg.data[6]    = static_cast<uint8_t>((id_s15_16 >> 16) & 0xFF);
  tx_msg.data[7]    = static_cast<uint8_t>((id_s15_16 >> 24) & 0xFF);
}

void make_tx_msg(const joint_cmd &joint) {
  if(joint.torque_on == 0 && joint.motor_enabled == 0) { // torque_onが来ておらず、モータもenableじゃないので何もしない
    debugPrint(DEBUG_INFO, "Processing joint ID: 0x%X - Case A (Torque Off, Motor Disabled)", joint.joint_id);
    make_tx_header(joint.joint_id, cmd_id::TEST_COMMAND, cmd_id::WITHOUT_RES);
    make_tx_data_fill_zero();

  } else if(joint.torque_on == 1 && joint.motor_enabled == 0) { // torque_onが来て、モータがenableじゃないのでtorque_on
    debugPrint(DEBUG_INFO, "Processing joint ID: 0x%X - Case B (Torque On, Motor Disabled)", joint.joint_id);
    make_tx_header(joint.joint_id, cmd_id::TORQUE_ON);
    make_tx_data_fill_zero();

  } else if(joint.torque_on == 0 && joint.motor_enabled == 1) { // torque_onが消えて、モータがenableなのでtorque_off
    debugPrint(DEBUG_INFO, "Processing joint ID: 0x%X - Case D (Torque Off, Motor Enabled)", joint.joint_id);
    make_tx_header(joint.joint_id, cmd_id::TORQUE_OFF);
    make_tx_data_fill_zero();

  } else if(joint.torque_on == 1 && joint.motor_enabled == 1) { // モータがenableなのでmove_angleかSetTargetCurrentを実行
    debugPrint(DEBUG_INFO, "Processing joint ID: 0x%X - Case C (Torque On, Motor Enabled)", joint.joint_id);
    if(joint.control_mode == 0) {
      debugPrint(DEBUG_INFO, "Joint ID: 0x%X - Control Mode: Move Angle", joint.joint_id);
      make_tx_header(joint.joint_id, cmd_id::MOVE_ANGLE);
      make_tx_data_move_angle(joint.target_angle, joint.move_time, joint.current_limit);
    } else if(joint.control_mode == 1) {
      debugPrint(DEBUG_INFO, "Joint ID: 0x%X - Control Mode: Set Target Current", joint.joint_id);
      make_tx_header(joint.joint_id, cmd_id::SET_TARGET_CURRENT);
      make_tx_data_set_target_current(joint.target_iq, joint.target_id);
    }
  }
}
void clear_tx_msg(twai_message_t &tx_msg) {
  tx_msg.identifier       = 0;
  tx_msg.extd             = 0;
  tx_msg.rtr              = 0;
  tx_msg.data_length_code = 0;
  for(int i = 0; i < 8; ++i) {
    tx_msg.data[i] = 0;
  }
}

void print_tx_msg(const twai_message_t &msg) {
  Serial.println("---- TX Message ----");
  Serial.print("Identifier: 0x");
  Serial.println(msg.identifier, HEX);
  Serial.print("Extended ID: ");
  Serial.println(msg.extd ? "Yes" : "No");
  Serial.print("DLC (Data Length Code): ");
  Serial.println(msg.data_length_code);
  Serial.print("Data: ");
  for(uint8_t i = 0; i < msg.data_length_code; ++i) {
    Serial.print("0x");
    Serial.print(msg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("---------------------");
}

const joint_cmd *find_joint_cmd_by_id(const std::vector<joint_cmd> &joint_list, uint8_t joint_id) {
  for(const auto &joint : joint_list) {
    if(joint.joint_id == joint_id) {
      return &joint; // 一致する joint_cmd のアドレス
    }
  }
  return nullptr; // 見つからない場合は nullptr
}
void handleCanAlerts() {
  uint32_t alerts;
  if(twai_read_alerts(&alerts, pdMS_TO_TICKS(10)) == ESP_OK) {
    if(alerts & TWAI_ALERT_TX_IDLE) {
      debugPrint(DEBUG_WARN, "CAN Alert: TX Idle - No messages to transmit.");
    }
    if(alerts & TWAI_ALERT_TX_SUCCESS) {
      debugPrint(DEBUG_WARN, "CAN Alert: TX Success - Previous transmission successful.");
    }
    if(alerts & TWAI_ALERT_RX_DATA) {
      debugPrint(DEBUG_WARN, "CAN Alert: RX Data - A frame has been received.");
    }
    if(alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
      debugPrint(DEBUG_WARN, "CAN Alert: Error Warning - Error counter exceeded warning limit.");
    }
    if(alerts & TWAI_ALERT_BUS_ERROR) {
      debugPrint(DEBUG_WARN, "CAN Alert: Bus Error - Bus-level error occurred.");
    }
    if(alerts & TWAI_ALERT_TX_FAILED) {
      debugPrint(DEBUG_WARN, "CAN Alert: TX Failed - Previous transmission failed.");
    }
    if(alerts & TWAI_ALERT_BUS_OFF) {
      debugPrint(DEBUG_WARN, "CAN Alert: Bus Off - TWAI controller entered bus-off state.");
    }
    if(alerts & TWAI_ALERT_RX_QUEUE_FULL) {
      debugPrint(DEBUG_WARN, "CAN Alert: RX Queue Full - RX queue overflow.");
    }
    if(alerts & TWAI_ALERT_RX_FIFO_OVERRUN) {
      debugPrint(DEBUG_WARN, "CAN Alert: RX FIFO Overrun - RX FIFO overflow.");
    }
  } else {
    debugPrint(DEBUG_INFO, "No CAN alerts received.");
  }
}
void can_task_main() {
  setDebugLevel(CAN_DEBUG_LEVEL_SETTING);
  for(size_t i = 0; i < 12; ++i) {
    joint_cmd &joint = joint_cmd_list[i];
    make_tx_msg(joint);
    // 送信
    if(twai_transmit(&tx_msg, pdMS_TO_TICKS(1)) == ESP_OK) {
      debugPrint(DEBUG_INFO, "CAN message sent. ID: 0x%X", joint.joint_id);
    } else {
      debugPrint(DEBUG_ERROR, "Failed to send CAN message. ID: 0x%X", joint.joint_id);
      uint32_t base_id = (tx_msg.identifier >> 18) & 0x7FF;
      // handleTransmitError(base_id, tx_msg);
      debugPrint(DEBUG_INFO, "Clearing TX queue...");
      if(twai_clear_transmit_queue() != ESP_OK) {
        debugPrint(DEBUG_ERROR, "Failed to clear TX queue.");
        return;
      }
      debugPrint(DEBUG_INFO, "Processing joint ID: 0x%X", joint.joint_id);
    }
    // 受信
    twai_message_t rx_message;
    if(twai_receive(&rx_msg, pdMS_TO_TICKS(1)) == ESP_OK) {
      // handleCanAlerts(); // アラート内容を出力。これを入れないとRX error counter が125位で死にそうになる
      delayMicroseconds(1000); // 代替おまじない。ちょっと成功確率ｱﾔﾚｲ。MDのリセット→ESPのリセットを順守すること。
      processResSummary(rx_msg);
    } else {
      debugPrint(DEBUG_ERROR, "Failed to recieve CAN message. ID: 0x%X", joint.joint_id);
    }
  }
  return;
}