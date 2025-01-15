#ifndef JOINT_DATA_CONTROL_HPP
#define JOINT_DATA_CONTROL_HPP

#include <Arduino.h>
#include <cstdint>
#include <vector>

// 上下限値設定。今は適当
#define ANGLE_MIN -180.0f * 30
#define ANGLE_MAX 180.0f * 30
#define MOVE_TIME_MIN 0
#define MOVE_TIME_MAX 10000
#define CURRENT_LIMIT_MIN 0.0f
#define CURRENT_LIMIT_MAX 10.0f
#define TARGET_IQ_MIN -24.0f
#define TARGET_IQ_MAX 24.0f
#define TARGET_ID_MIN -24.0f
#define TARGET_ID_MAX 24.0f

enum class cmd_id : uint16_t { // コマンド定義
  TEST_COMMAND        = 0x1234,
  TORQUE_ON           = 0x0001,
  TORQUE_OFF          = 0x0002,
  TORQUE_CONTROL_MODE = 0x0003,
  MOVE_ANGLE          = 0x0010,
  SET_TARGET_CURRENT  = 0x0110,
  WITH_RES            = 0x8000,
  WITHOUT_RES         = 0x0000
};

enum joint_id : uint8_t { // 関節ID定義
  ID_FR_THIGH_ROLL  = 0x31,
  ID_FR_THIGH_PITCH = 0x32,
  ID_FR_THIGH_KNEE  = 0x33,
  ID_FL_THIGH_ROLL  = 0x21,
  ID_FL_THIGH_PITCH = 0x22,
  ID_FL_THIGH_KNEE  = 0x23,
  ID_RR_THIGH_ROLL  = 0x01,
  ID_RR_THIGH_PITCH = 0x02,
  ID_RR_THIGH_KNEE  = 0x03,
  ID_RL_THIGH_ROLL  = 0x11,
  ID_RL_THIGH_PITCH = 0x12,
  ID_RL_THIGH_KNEE  = 0x13
};

struct joint_cmd {        // 関節指示を表す構造体
  uint8_t  joint_id;      // 関節ID
  bool     torque_on;     // トルクオンコマンド有無
  bool     motor_enabled; // モーター有効状態
  uint32_t control_mode;  // 制御モード
  float    target_angle;  // 目標角度
  uint32_t move_time;     // 移動時間
  float    current_limit; // 電流制限
  float    target_iq;     // 目標Iq
  float    target_id;     // 目標Id

  joint_cmd(uint8_t id); // 初期化
};

struct joint_state {     // 関節状態を表す構造体
  uint8_t joint_id;      // 関節ID
  float   angle;         // 現在角度
  float   current;       // モータ電流
  float   motor_voltage; // モータ電圧
  float   vm_voltage;    // VM電圧

  joint_state(uint8_t id); // 初期化
};

extern std::vector<joint_state> joint_state_list; // moteusから来たのを格納
extern std::vector<joint_cmd>   joint_cmd_list;   // PCから送られてきたのを格納

std::vector<joint_state> initialize_joint_state_list(); // 関節情報リストの初期化
std::vector<joint_cmd>   initialize_joint_cmd_list();   // 関節情報リストの初期化

void print_joint_state(const std::vector<joint_state> &joints); // 関節情報をシリアル出力
void print_joint_cmd(const std::vector<joint_cmd> &joints);     // 関節情報をシリアル出力
void set_joint_command(                                         // 関節情報の格納
    std::vector<joint_state> &joint_cmd_list,
    uint8_t                   joint_id,
    float                     target_angle,
    uint32_t                  move_time,
    float                     current_limit,
    float                     target_iq,
    float                     target_id);

#endif // JOINT_DATA_CONTROL_HPP