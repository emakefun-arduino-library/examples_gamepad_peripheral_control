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

#include "codex_pad.h"
#include "motor.h"
#include "cyf.h"
#include "cyf/log.h"

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
const std::string kBluetoothDeviceAddress = "16:00:00:00:19:94";

constexpr gpio_num_t kDcMotor0PinPositivePin = GPIO_NUM_27;  // 直流电机0正极引脚。 | DC Motor 0 positive pin.
constexpr gpio_num_t kDcMotor0PinNegativePin = GPIO_NUM_13;  // 直流电机0负极引脚。 | DC Motor 0 negative pin.

constexpr gpio_num_t kDcMotor1PinPositivePin = GPIO_NUM_4;  // 直流电机1正极引脚。 | DC Motor 1 positive pin.
constexpr gpio_num_t kDcMotor1PinNegativePin = GPIO_NUM_2;  // 直流电机1负极引脚。 | DC Motor 1 negative pin.

constexpr gpio_num_t kDcMotor2PinPositivePin = GPIO_NUM_17;  // 直流电机2正极引脚。 | DC Motor 2 positive pin.
constexpr gpio_num_t kDcMotor2PinNegativePin = GPIO_NUM_12;  // 直流电机2负极引脚。 | DC Motor 2 negative pin.

constexpr gpio_num_t kDcMotor3PinPositivePin = GPIO_NUM_15;  // 直流电机3正极引脚。 | DC Motor 3 positive pin.
constexpr gpio_num_t kDcMotor3PinNegativePin = GPIO_NUM_14;  // 直流电机3负极引脚。 | DC Motor 3 negative pin.

constexpr uint32_t kMotorUpdateIntervalMs = 100;  // 电机更新间隔时间，单位毫秒。 | Motor update interval in milliseconds.

// 连接手柄超时时间，单位毫秒。 | Connection timeout for gamepad in milliseconds.
constexpr uint32_t kConnectionTimeoutMs = 5000;

uint32_t g_motor_last_update_time = 0;

// 四个直流电机对象。 | Four DC motor objects.
em::Motor g_dc_motor_0(kDcMotor0PinPositivePin, kDcMotor0PinNegativePin);
em::Motor g_dc_motor_1(kDcMotor1PinPositivePin, kDcMotor1PinNegativePin);
em::Motor g_dc_motor_2(kDcMotor2PinPositivePin, kDcMotor2PinNegativePin);
em::Motor g_dc_motor_3(kDcMotor3PinPositivePin, kDcMotor3PinNegativePin);

codex_pad::Client g_codex_pad_client;

void Connect() {
  CLOGI("Start to connect %s", kBluetoothDeviceAddress.c_str());
  // Connect to the CodexPad with specified Bluetooth device address
  // 连接到指定蓝牙设备地址的手柄
  while (!g_codex_pad_client.Connect(kBluetoothDeviceAddress, 5000)) {
    CLOGI("Retry to connect %s", kBluetoothDeviceAddress.c_str());
  }

  CLOGI("Remote device name: %s", g_codex_pad_client.remote_device_name().c_str());
  CLOGI("Remote model number: %s", g_codex_pad_client.remote_model_number().c_str());
  CLOGI("Remote firmware revision: %u.%u.%u", g_codex_pad_client.remote_firmware_version()[0],
        g_codex_pad_client.remote_firmware_version()[1], g_codex_pad_client.remote_firmware_version()[2]);

  if (const auto ble_client = g_codex_pad_client.ble_client(); ble_client != nullptr) {
    CLOGI("Remote Bluetooth Device Address: %s", ble_client->getPeerAddress().toString().c_str());
  } else {
    CLOGI("Remote Bluetooth Device Address: unknown");
  }

  CLOGI("Connected");
}

void MotorInit() {
  CLOGI("Motor driver init.\n");
  g_dc_motor_0.Init();
  g_dc_motor_1.Init();
  g_dc_motor_2.Init();
  g_dc_motor_3.Init();

  g_dc_motor_0.RunPwmDuty(0);
  g_dc_motor_1.RunPwmDuty(0);
  g_dc_motor_2.RunPwmDuty(0);
  g_dc_motor_3.RunPwmDuty(0);
}
}  // namespace

void setup() {
  MotorInit();

  CLOGI("CodexPad Init.\n");
  g_codex_pad_client.Init();

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
  const gamepad::input::Tracker& it = g_codex_pad_client.Update();
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

  if (!g_codex_pad_client.is_connected()) {
    CLOGI("Disconnected, start to reconnect.\n");
    Connect();
    return;
  }

  if (millis() - g_motor_last_update_time >= kMotorUpdateIntervalMs) {
    if (it.holding(Button::kCrossA)) {
      // Cross按钮：刹车 | Cross button: brake.
      CLOGI("Brake.\n");
      g_dc_motor_0.Stop();
      g_dc_motor_1.Stop();
      g_dc_motor_2.Stop();
      g_dc_motor_3.Stop();
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kUp) && !it.holding(Button::kDown)) {
      // 上方向键：电机正转 | Up button: move forward.
      CLOGI("Forward.\n");
      g_dc_motor_0.RunPwmDuty(em::Motor::kMaxPwmDuty);
      g_dc_motor_1.RunPwmDuty(em::Motor::kMaxPwmDuty);
      g_dc_motor_2.RunPwmDuty(em::Motor::kMaxPwmDuty);
      g_dc_motor_3.RunPwmDuty(em::Motor::kMaxPwmDuty);
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kDown) && !it.holding(Button::kUp)) {
      // 下方向键：电机反转 | Down button: move backward.
      CLOGI("Reverse.\n");
      g_dc_motor_0.RunPwmDuty(-em::Motor::kMaxPwmDuty);
      g_dc_motor_1.RunPwmDuty(-em::Motor::kMaxPwmDuty);
      g_dc_motor_2.RunPwmDuty(-em::Motor::kMaxPwmDuty);
      g_dc_motor_3.RunPwmDuty(-em::Motor::kMaxPwmDuty);
      g_motor_last_update_time = millis();
    } else {
      // 无输入：停止 | No input: stop.
      CLOGI("Stop.\n");
      g_dc_motor_0.RunPwmDuty(0);
      g_dc_motor_1.RunPwmDuty(0);
      g_dc_motor_2.RunPwmDuty(0);
      g_dc_motor_3.RunPwmDuty(0);
      g_motor_last_update_time = millis();
    }
  }
}