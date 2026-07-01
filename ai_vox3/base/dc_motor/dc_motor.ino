/**
 * @~English
 * @brief Control 4 DC motors using CodexPad-S10 gamepad.
 * @details Control the running state of 4 DC motors via gamepad buttons:
 *         - Up button (holding): All 4 motors run forward (PWM duty 1023).
 *         - Down button (holding): All 4 motors run backward (PWM duty -1023).
 *         - Cross button (holding): Brake (call Stop method), brake has the highest priority.
 *         - No button input: All motors stop (PWM duty 0).
 */
/**
 * @~Chinese
 * @brief 使用CodexPad-S10手柄控制4个直流电机。
 * @details 通过手柄按钮控制4个直流电机的运行状态：
 *          - 上方向按钮（按住）：4个电机同时正转（PWM占空比 1023）。
 *          - 下方向按钮（按住）：4个电机同时反转（PWM占空比 -1023）。
 *          - Cross按钮（按住）：刹车（调用Stop方法），刹车优先级最高。
 *          - 无按钮输入时，所有电机停止（PWM占空比 0）。
 */

#include <Wire.h>

#include "codex_pad.h"
#include "md40.h"

/**
 * IMPORTANT:
 * This using directive is REQUIRED to directly access `Button` and `Axis`.
 * Without it, you must write the fully qualified names:
 *   gamepad::input::Button::kUp
 *   gamepad::input::Axis::kLeftStickX
 * Forgetting this line will cause compile errors when using Button or Axis.
 */
/**
 * 重要：
 * 必须使用该命名空间，否则无法直接访问 `Button` 和 `Axis`。
 * 如果没有这一行，就必须写成完整限定名：
 *   gamepad::input::Button::kUp
 *   gamepad::input::Axis::kLeftStickX
 * 忘记引入命名空间会导致编译失败。
 */
using namespace gamepad::input;

namespace {
// Replace with your CodexPad device's Bluetooth device address.
// 替换为你的 CodexPad 的 Bluetooth device address。
const std::string kBluetoothDeviceAddress = "16:00:00:00:03:27";

constexpr gpio_num_t kI2cPinSda = GPIO_NUM_13;  // I2C 数据线引脚。 | I2C data pin.
constexpr gpio_num_t kI2cPinScl = GPIO_NUM_12;  // I2C 时钟线引脚。 | I2C clock pin.

constexpr uint16_t kMotorRunPwmDuty = 1023;       // 电机运行时的PWM占空比。 | PWM duty when motor is running.
constexpr uint32_t kMotorUpdateIntervalMs = 100;  // 电机更新间隔时间，单位毫秒。 | Motor update interval in milliseconds.

// 连接手柄超时时间，单位毫秒。 | Connection timeout for gamepad in milliseconds.
constexpr uint32_t kConnectionTimeoutMs = 5000;

uint32_t g_motor_last_update_time = 0;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

CodexPad g_codex_pad;

void Connect() {
  printf("Start to connect %s\n", kBluetoothDeviceAddress.c_str());
  // Connect to the CodexPad with specified Bluetooth device address.
  // 连接到指定蓝牙设备地址的手柄。
  while (!g_codex_pad.Connect(kBluetoothDeviceAddress, kConnectionTimeoutMs)) {
    printf("Retry to connect %s\n", kBluetoothDeviceAddress.c_str());
  }

  if (const auto ble_client = g_codex_pad.ble_client(); ble_client != nullptr) {
    printf("Remote Bluetooth Device Address: %s\n", ble_client->getPeerAddress().toString().c_str());
  } else {
    printf("Remote Bluetooth Device Address: unknown.\n");
  }

  // Set transmission power to 0dBm.
  // Transmission power affects communication range and power consumption:
  // Higher power provides longer range but consumes more battery.
  // Choose appropriate power level based on your application to balance range and battery life.
  // 设置发射功率为0dBm。
  // 发射功率影响通信距离和功耗：功率越高，通信距离越远，但功耗也越大。
  // 建议根据实际应用场景选择合适的功率等级以平衡距离和电池寿命。
  if (g_codex_pad.set_remote_tx_power(CodexPad::TxPower::k0dBm)) {
    printf("Set remote tx power to 0dBm successfully.\n");
  }

  printf("Connected.\n");
}

void Md40Init() {
  printf("Md40 init, dc motor mode.\n");
  g_md40.Init();

  // 设置Md40运行模式为直流电机模式，并初始停止。 | Set Md40 to run in DC motor mode and initially stop.
  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    g_md40[i].SetDcMode();
    g_md40[i].RunPwmDuty(0);
  }
}
}  // namespace

void setup() {
  Wire.begin(kI2cPinSda, kI2cPinScl);

  Md40Init();

  printf("CodexPad Init.\n");
  g_codex_pad.Init();

  Connect();
}

void loop() {
  // ==========================================================================
  // 🔴 CRITICAL: Call Update() as frequently as possible in loop()
  // ==========================================================================
  // • Update() processes incoming Bluetooth packets from the CodexPad
  // • Any delay(...) or long blocking code WILL cause:
  //     - Packet loss
  //     - Input lag
  //     - Unstable connection
  //
  // • For real-time control, call Update() every loop iteration
  //   without any blocking operations
  //
  // 🔴【重要】Update() 必须在 loop() 中尽可能高频调用
  // • Update() 负责处理来自 CodexPad 的蓝牙数据包
  // • 任何形式的 delay 或阻塞代码都会导致：
  //     - 数据丢失
  //     - 响应延迟
  //     - 连接不稳定
  //
  // • 实时控制应用中，必须每轮循环都调用 Update()，不可阻塞
  // ==========================================================================
  const Tracker& it = g_codex_pad.Update();
  // ==========================================================================
  // Tracker: Gamepad Input Snapshot & Change Engine
  // ==========================================================================
  // • Returned by Update()
  // • Maintains previous and current input snapshots
  // • Enables edge detection and delta detection
  //
  // Tracker 由 Update() 返回
  // 内部保存上一帧和当前帧的输入数据
  // 支持边沿检测和差值检测
  //
  // 📚 https://codexpad.github.io/gamepad_input_arduino_lib/
  // ==========================================================================

  if (!g_codex_pad.is_connected()) {
    printf("Disconnected, start to reconnect.\n");
    Connect();
    return;
  }

  if (millis() - g_motor_last_update_time >= kMotorUpdateIntervalMs) {
    if (it.holding(Button::kCrossA)) {
      // Cross按钮：刹车 | Cross button: brake.
      printf("Brake.\n");
      for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
        g_md40[i].Stop();
      }
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kUp) && !it.holding(Button::kDown)) {
      // 上方向键：电机正转 | Up button: move forward.
      printf("Forward.\n");
      for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
        g_md40[i].RunPwmDuty(kMotorRunPwmDuty);
      }
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kDown) && !it.holding(Button::kUp)) {
      // 下方向键：电机反转 | Down button: move backward.
      printf("Reverse.\n");
      for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
        g_md40[i].RunPwmDuty(-kMotorRunPwmDuty);
      }
      g_motor_last_update_time = millis();
    } else {
      // 无输入：停止 | No input: stop.
      printf("Stop.\n");
      for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
        g_md40[i].RunPwmDuty(0);
      }
      g_motor_last_update_time = millis();
    }
  }
}