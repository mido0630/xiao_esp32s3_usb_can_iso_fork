#include "joint_data_control.hpp"
#include "debug_print.hpp"

#define JDC_DEBUG_LEVEL_SETTING DEBUG_WARN // デバッグレベル設定

joint_cmd::joint_cmd(uint8_t id)
    : joint_id(id), torque_on(false), motor_enabled(false), control_mode(0), target_angle(0.0f), move_time(0), current_limit(0.0f), target_iq(0.0f), target_id(0.0f) {}

joint_state::joint_state(uint8_t id)
    : joint_id(id), angle(0.0f), current(0.0f), motor_voltage(0.0f), vm_voltage(0.0f) {}

std::vector<joint_state> initialize_joint_state_list() { // 関節状態リストの初期化
  return {
      joint_state(ID_FR_THIGH_ROLL),
      joint_state(ID_FR_THIGH_PITCH),
      joint_state(ID_FR_THIGH_KNEE),
      joint_state(ID_FL_THIGH_ROLL),
      joint_state(ID_FL_THIGH_PITCH),
      joint_state(ID_FL_THIGH_KNEE),
      joint_state(ID_RR_THIGH_ROLL),
      joint_state(ID_RR_THIGH_PITCH),
      joint_state(ID_RR_THIGH_KNEE),
      joint_state(ID_RL_THIGH_ROLL),
      joint_state(ID_RL_THIGH_PITCH),
      joint_state(ID_RL_THIGH_KNEE)};
}
std::vector<joint_cmd> initialize_joint_cmd_list() { // 関節指示リストの初期化
  return {
      joint_cmd(ID_FR_THIGH_ROLL),
      joint_cmd(ID_FR_THIGH_PITCH),
      joint_cmd(ID_FR_THIGH_KNEE),
      joint_cmd(ID_FL_THIGH_ROLL),
      joint_cmd(ID_FL_THIGH_PITCH),
      joint_cmd(ID_FL_THIGH_KNEE),
      joint_cmd(ID_RR_THIGH_ROLL),
      joint_cmd(ID_RR_THIGH_PITCH),
      joint_cmd(ID_RR_THIGH_KNEE),
      joint_cmd(ID_RL_THIGH_ROLL),
      joint_cmd(ID_RL_THIGH_PITCH),
      joint_cmd(ID_RL_THIGH_KNEE)};
}

std::vector<joint_state> joint_state_list = initialize_joint_state_list();
std::vector<joint_cmd>   joint_cmd_list   = initialize_joint_cmd_list();

void print_joint_cmd(const std::vector<joint_cmd> &joints) { // 関節指示リストのシリアル出力
  Serial.println("--- Joint Cmd List ---");
  for(const auto &joint : joints) {
    Serial.print("ID: 0x");
    Serial.print(joint.joint_id, HEX);
    Serial.print(",\tTorque On: ");
    Serial.print(joint.torque_on ? "Yes" : "No");
    Serial.print(",\tMotor Enabled: ");
    Serial.print(joint.motor_enabled ? "Yes" : "No");
    Serial.print(",\tControl Mode: ");
    Serial.print(joint.control_mode);
    Serial.print(",\tTarget Angle: ");
    Serial.print(joint.target_angle);
    Serial.print(",\tMove Time: ");
    Serial.print(joint.move_time);
    Serial.print(",\tCurrent Limit: ");
    Serial.print(joint.current_limit);
    Serial.print(",\tTarget Iq: ");
    Serial.print(joint.target_iq);
    Serial.print(",\tTarget Id: ");
    Serial.println(joint.target_id);
  }
  Serial.println("-------------------");
}
void print_joint_state(const std::vector<joint_state> &joints) { // 関節状態リストのシリアル出力
  Serial.println("--- Joint Cmd List ---");
  Serial.println("--- Joint State List ---");
  for(const auto &joint : joints) {
    Serial.print("ID: 0x");
    Serial.print(joint.joint_id, HEX);
    Serial.print(",\tAngle: ");
    Serial.print(joint.angle, 2);
    Serial.print(" deg");
    Serial.print(",\tCurrent: ");
    Serial.print(joint.current, 3);
    Serial.print(" A");
    Serial.print(",\tMotor Voltage: ");
    Serial.print(joint.motor_voltage, 3);
    Serial.print(" V");
    Serial.print(",\tVM Voltage: ");
    Serial.print(joint.vm_voltage, 3);
    Serial.println(" V");
  }
  Serial.println("-------------------");
}

float crop_float(float value, float min_value, float max_value) {
  if(value < min_value) return min_value;
  if(value > max_value) return max_value;
  return value;
}

uint32_t crop_uint32(uint32_t value, uint32_t min_value, uint32_t max_value) {
  if(value < min_value) return min_value;
  if(value > max_value) return max_value;
  return value;
}

void set_joint_command( // joint_cmd_list に書き込む関数
    std::vector<joint_cmd> &joint_cmd_list,
    uint8_t                 joint_id,
    float                   target_angle,
    uint32_t                move_time,
    float                   current_limit,
    float                   target_iq,
    float                   target_id) {
  for(auto &joint : joint_cmd_list) {
    if(joint.joint_id == joint_id) {
      joint.target_angle  = crop_float(target_angle, ANGLE_MIN, ANGLE_MAX);
      joint.move_time     = crop_uint32(move_time, MOVE_TIME_MIN, MOVE_TIME_MAX);
      joint.current_limit = crop_float(current_limit, CURRENT_LIMIT_MIN, CURRENT_LIMIT_MAX);
      joint.target_iq     = crop_float(target_iq, TARGET_IQ_MIN, TARGET_IQ_MAX);
      joint.target_id     = crop_float(target_id, TARGET_ID_MIN, TARGET_ID_MAX);

      debugPrint(DEBUG_INFO, "Updated joint ID: 0x%X", joint_id);
      debugPrint(DEBUG_DETAIL, "Target Angle: %.2f, Move Time: %u, Current Limit: %.2f, Target Iq: %.2f, Target Id: %.2f",
                 joint.target_angle, joint.move_time, joint.current_limit, joint.target_iq, joint.target_id);
      return;
    }
  }
  debugPrint(DEBUG_WARN, "Joint ID: 0x%X not found in joint_cmd_list", joint_id);
}
