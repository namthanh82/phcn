#include "ODriveCAN.h"

// Tốc độ Baud mạng CAN
#define CAN_BAUDRATE 250000

#define ODRV0_NODE_ID 0  // Động cơ 1
#define ODRV1_NODE_ID 1  // Động cơ 2

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

// Chân GPIO trên ESP32-S3 cắm nối với mạch SN65HVD230
#define ESP32_TWAI_TX_PIN 1   // Dây CTX 
#define ESP32_TWAI_RX_PIN 2   // Dây CRX 

ESP32TWAIIntf can_intf;

bool setupCan() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)ESP32_TWAI_TX_PIN,
        (gpio_num_t)ESP32_TWAI_RX_PIN,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    return true;
}
#endif // IS_ESP32_TWAI

// Khởi tạo Object CAN
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
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

// -----------------------------------------------------------------------------------
// HÀM XỬ LÝ LỆNH TỪ SERIAL (MÁY TÍNH GỬI XUỐNG)
// -----------------------------------------------------------------------------------
void processSerialCommand() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  auto applyCommand = [](ODriveCAN& odrv, const String& body) {
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
    applyCommand(odrv0, cmd.substring(3));
    Serial.print("CMD P0: ");
    Serial.println(cmd.substring(3));
  } else if (cmd.startsWith("P1:")) {
    applyCommand(odrv1, cmd.substring(3));
    Serial.print("CMD P1: ");
    Serial.println(cmd.substring(3));
  } else if (cmd.startsWith("T0:")) {
    Serial.println("WARN: Torque mode command received, but this firmware is using setPosition().");
  } else if (cmd.startsWith("T1:")) {
    Serial.println("WARN: Torque mode command received, but this firmware is using setPosition().");
  } else if (cmd == "PING") {
    Serial.println("PONG");
  }
}

void setup() {
  // SET TỐC ĐỘ SERIAL LÊN CAO ĐỂ TRUYỀN NHANH, KHÔNG TRỄ
  Serial.begin(115200); 

  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  Serial.println("INFO: ESP32-S3 Bridge Khởi Động.");

  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  if (!setupCan()) {
    Serial.println("ERROR: CAN init failed!");
    while (true); 
  }

  // Đợi kết nối với Node 0
  Serial.println("INFO: Waiting for Node 0...");
  while (!odrv0_user_data.received_heartbeat) pumpEvents(can_intf);
  
  // Đợi kết nối với Node 1
  Serial.println("INFO: Waiting for Node 1...");
  while (!odrv1_user_data.received_heartbeat) pumpEvents(can_intf);

  // Kích hoạt CLOSED LOOP cho Node 0
  Serial.println("INFO: Enabling Closed Loop for Node 0...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    for(int i=0; i<15; i++) { delay(10); pumpEvents(can_intf); }
  }

  // Kích hoạt CLOSED LOOP cho Node 1
  Serial.println("INFO: Enabling Closed Loop for Node 1...");
  while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv1.clearErrors();
    delay(1);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    for(int i=0; i<15; i++) { delay(10); pumpEvents(can_intf); }
  }

  Serial.println("READY: System is ready to receive commands.");
}

void loop() {
  pumpEvents(can_intf); // Liên tục bơm dữ liệu CAN

  // 1. Kiểm tra và thực thi tín hiệu Máy tính nạp xuống
  processSerialCommand();

  // 2. Gom dữ liệu Feedback từ ODrive gửi lên Máy tính để vẽ biểu đồ
  if (odrv0_user_data.received_feedback && odrv1_user_data.received_feedback) {
    float pos0 = odrv0_user_data.last_feedback.Pos_Estimate;
    float pos1 = odrv1_user_data.last_feedback.Pos_Estimate;
    
    odrv0_user_data.received_feedback = false;
    odrv1_user_data.received_feedback = false;
    Serial.print("FB,");
    Serial.print(pos0);
    Serial.print(",");
    Serial.println(pos1);
  }
}
