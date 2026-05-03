#include "ODriveCAN.h"

// CAN baudrate
#define CAN_BAUDRATE 250000

#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1

#define IS_ESP32_TWAI

#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) + defined(IS_STM32_BUILTIN) + defined(IS_ESP32_TWAI) != 1
#warning "Select exactly one hardware option at the top of this file."
#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif
#endif

#ifdef IS_ESP32_TWAI
#include "driver/twai.h"
#include "ODriveESP32TWAI.hpp"

// ESP32-S3 + SN65HVD230
#define ESP32_TWAI_TX_PIN 1
#define ESP32_TWAI_RX_PIN 2

ESP32TWAIIntf can_intf;

bool setupCan() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)ESP32_TWAI_TX_PIN,
      (gpio_num_t)ESP32_TWAI_RX_PIN,
      TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  return true;
}
#endif

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

bool closed_loop_requested = false;
unsigned long last_fb_ms = 0;

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

bool waitHeartbeat(ODriveUserData& ud, const char* label, uint32_t timeout_ms) {
  Serial.print("INFO: Waiting heartbeat ");
  Serial.println(label);
  unsigned long t0 = millis();
  while (!ud.received_heartbeat) {
    pumpEvents(can_intf);
    if ((millis() - t0) > timeout_ms) {
      Serial.print("ERROR: heartbeat timeout ");
      Serial.println(label);
      return false;
    }
    delay(2);
  }
  return true;
}

void setClosedLoop(bool enable) {
  uint32_t target = enable ? ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL : ODRIVE_AXIS_STATE_IDLE;
  sendAxisState(ODRV0_NODE_ID, target);
  sendAxisState(ODRV1_NODE_ID, target);
  closed_loop_requested = enable;
  Serial.print("INFO: state request=");
  Serial.println(enable ? "CLOSED_LOOP" : "IDLE");
}

void setTorqueMode() {
  sendControllerMode(ODRV0_NODE_ID, ODRIVE_CONTROL_MODE_TORQUE_CONTROL, ODRIVE_INPUT_MODE_PASSTHROUGH);
  sendControllerMode(ODRV1_NODE_ID, ODRIVE_CONTROL_MODE_TORQUE_CONTROL, ODRIVE_INPUT_MODE_PASSTHROUGH);
  Serial.println("INFO: controller mode=TORQUE/PASSTHROUGH");
}

void setPositionMode() {
  sendControllerMode(ODRV0_NODE_ID, ODRIVE_CONTROL_MODE_POSITION_CONTROL, ODRIVE_INPUT_MODE_PASSTHROUGH);
  sendControllerMode(ODRV1_NODE_ID, ODRIVE_CONTROL_MODE_POSITION_CONTROL, ODRIVE_INPUT_MODE_PASSTHROUGH);
  Serial.println("INFO: controller mode=POSITION/PASSTHROUGH");
}

void clearBothErrors() {
  odrv0.clearErrors();
  odrv1.clearErrors();
  Serial.println("INFO: clearErrors sent");
}

void printStatus() {
  Serial.print("STATUS,HB0=");
  Serial.print(odrv0_user_data.received_heartbeat ? 1 : 0);
  Serial.print(",HB1=");
  Serial.print(odrv1_user_data.received_heartbeat ? 1 : 0);
  Serial.print(",REQ=");
  Serial.print(closed_loop_requested ? "CLOSED_LOOP" : "IDLE");
  Serial.print(",AX0=");
  Serial.print((int)odrv0_user_data.last_heartbeat.Axis_State);
  Serial.print(",AX1=");
  Serial.println((int)odrv1_user_data.last_heartbeat.Axis_State);
}

void writeFloatLE(uint8_t* dst, float value) {
  memcpy(dst, &value, sizeof(float));
}

void writeU32LE(uint8_t* dst, uint32_t value) {
  memcpy(dst, &value, sizeof(uint32_t));
}

bool sendOdriveCan(uint8_t node_id, uint8_t cmd_id, const uint8_t* data, uint8_t dlc) {
  twai_message_t msg = {};
  msg.extd = 0;
  msg.rtr = 0;
  msg.ss = 0;
  msg.self = 0;
  msg.dlc_non_comp = 0;
  msg.identifier = ((uint32_t)node_id << 5) | cmd_id;
  msg.data_length_code = dlc;
  for (uint8_t i = 0; i < dlc && i < 8; ++i) {
    msg.data[i] = data[i];
  }
  return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
}

void sendAxisState(uint8_t node_id, uint32_t axis_state) {
  uint8_t data[4] = {0};
  writeU32LE(data, axis_state);
  if (!sendOdriveCan(node_id, ODRIVE_CMD_SET_AXIS_REQUESTED_STATE, data, 4)) {
    Serial.print("ERROR: Set_Axis_State TX failed node=");
    Serial.println(node_id);
  }
}

void sendControllerMode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode) {
  uint8_t data[8] = {0};
  writeU32LE(&data[0], control_mode);
  writeU32LE(&data[4], input_mode);
  if (!sendOdriveCan(node_id, ODRIVE_CMD_SET_CONTROLLER_MODES, data, 8)) {
    Serial.print("ERROR: Set_Controller_Modes TX failed node=");
    Serial.println(node_id);
  }
}

void sendInputTorque(uint8_t node_id, float torque_nm) {
  uint8_t data[4] = {0};
  writeFloatLE(data, torque_nm);
  if (!sendOdriveCan(node_id, ODRIVE_CMD_SET_INPUT_TORQUE, data, 4)) {
    Serial.print("ERROR: Set_Input_Torque TX failed node=");
    Serial.println(node_id);
  }
}

void sendInputPosition(uint8_t node_id, float pos_rev, float vel_ff_rev_s) {
  uint8_t data[8] = {0};
  writeFloatLE(&data[0], pos_rev);
  int16_t vel = (int16_t)constrain(vel_ff_rev_s * 1000.0f, -32768.0f, 32767.0f);
  int16_t torque_ff = 0;
  memcpy(&data[4], &vel, sizeof(int16_t));
  memcpy(&data[6], &torque_ff, sizeof(int16_t));
  if (!sendOdriveCan(node_id, ODRIVE_CMD_SET_INPUT_POS, data, 8)) {
    Serial.print("ERROR: Set_Input_Pos TX failed node=");
    Serial.println(node_id);
  }
}

void printHelp() {
  Serial.println("HELP: T0:<Nm> | T1:<Nm> | P0:<rev>[,<rev_s>] | P1:<rev>[,<rev_s>] | TORQUE | POSITION | CLOSE | IDLE | CLEAR | STATUS | PING");
}

void processSerialCommand() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  auto applyPositionCommand = [](ODriveCAN& odrv, const String& body) {
    int comma = body.indexOf(',');
    float pos = 0.0f;
    float vel = 0.0f;
    if (comma >= 0) {
      pos = body.substring(0, comma).toFloat();
      vel = body.substring(comma + 1).toFloat();
    } else {
      pos = body.toFloat();
    }
    odrv.setPosition(pos, vel);
  };

  if (cmd.startsWith("P0:")) {
    int comma = cmd.substring(3).indexOf(',');
    String body = cmd.substring(3);
    float pos = comma >= 0 ? body.substring(0, comma).toFloat() : body.toFloat();
    float vel = comma >= 0 ? body.substring(comma + 1).toFloat() : 0.0f;
    sendInputPosition(ODRV0_NODE_ID, pos, vel);
  } else if (cmd.startsWith("P1:")) {
    int comma = cmd.substring(3).indexOf(',');
    String body = cmd.substring(3);
    float pos = comma >= 0 ? body.substring(0, comma).toFloat() : body.toFloat();
    float vel = comma >= 0 ? body.substring(comma + 1).toFloat() : 0.0f;
    sendInputPosition(ODRV1_NODE_ID, pos, vel);
  } else if (cmd.startsWith("T0:")) {
    sendInputTorque(ODRV0_NODE_ID, cmd.substring(3).toFloat());
  } else if (cmd.startsWith("T1:")) {
    sendInputTorque(ODRV1_NODE_ID, cmd.substring(3).toFloat());
  } else if (cmd == "TORQUE") {
    setTorqueMode();
  } else if (cmd == "POSITION") {
    setPositionMode();
  } else if (cmd == "CLOSE") {
    clearBothErrors();
    setClosedLoop(true);
  } else if (cmd == "IDLE") {
    setClosedLoop(false);
  } else if (cmd == "CLEAR") {
    clearBothErrors();
  } else if (cmd == "STATUS") {
    printStatus();
  } else if (cmd == "HELP") {
    printHelp();
  } else if (cmd == "PING") {
    Serial.println("PONG");
  } else {
    Serial.print("WARN: unknown cmd: ");
    Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  Serial.println("INFO: ESP32-S3 TWAI bridge boot");

  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  if (!setupCan()) {
    Serial.println("ERROR: CAN init failed");
    while (true) delay(1000);
  }

  if (!waitHeartbeat(odrv0_user_data, "node0", 7000) || !waitHeartbeat(odrv1_user_data, "node1", 7000)) {
    Serial.println("ERROR: heartbeat wait failed");
    while (true) {
      pumpEvents(can_intf);
      delay(10);
    }
  }

  clearBothErrors();
  setTorqueMode();
  setClosedLoop(true);
  Serial.println("READY: bridge online");
  printHelp();
}

void loop() {
  pumpEvents(can_intf);
  processSerialCommand();

  if (odrv0_user_data.received_feedback && odrv1_user_data.received_feedback) {
    float pos0 = odrv0_user_data.last_feedback.Pos_Estimate;
    float pos1 = odrv1_user_data.last_feedback.Pos_Estimate;
    odrv0_user_data.received_feedback = false;
    odrv1_user_data.received_feedback = false;

    Serial.print("FB,");
    Serial.print(pos0, 6);
    Serial.print(",");
    Serial.println(pos1, 6);
    last_fb_ms = millis();
  }

  if (millis() - last_fb_ms > 2000) {
    Serial.println("WARN: feedback timeout >2s");
    last_fb_ms = millis();
  }
}
