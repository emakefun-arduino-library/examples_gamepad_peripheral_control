/**
 * @~English
 * @brief Control 4 encoder motors using CodexPad-S10 gamepad.
 * @details Control the running state of 4 encoder motors via gamepad buttons:
 *          - Up button (holding): All 4 motors run forward (PWM duty 1023).
 *          - Down button (holding): All 4 motors run backward (PWM duty -1023).
 *          - Cross button (holding): Brake (call Stop method), brake has the highest priority.
 *          - No button input: All motors stop (PWM duty 0).
 */
/**
 * @~Chinese
 * @brief 使用CodexPad-S10手柄控制4个编码电机。
 * @details 通过手柄按钮控制4个编码电机的运行状态：
 *          - 上方向按钮（按住）：4个电机同时正转（PWM占空比 1023）。
 *          - 下方向按钮（按住）：4个电机同时反转（PWM占空比 -1023）。
 *          - Cross按钮（按住）：刹车（调用Stop方法），刹车优先级最高。
 *          - 无按钮输入时，所有电机停止（PWM占空比 0）。
 */

#include "codex_pad.h"
#include "encoder_motor.h"
#include "motor.h"

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

constexpr gpio_num_t kEncoderMotor0PositivePin = GPIO_NUM_27;  // 编码电机0正极引脚。 | Encoder motor 0 positive pin.
constexpr gpio_num_t kEncoderMotor0NegativePin = GPIO_NUM_13;  // 编码电机0负极引脚。 | Encoder motor 0 negative pin.
// 编码电机0编码器A相引脚。 | Encoder motor 0 encoder phase A pin.
constexpr gpio_num_t kEncoderMotor0EncoderPinA = GPIO_NUM_18;
// 编码电机0编码器B相引脚。 | Encoder motor 0 encoder phase B pin.
constexpr gpio_num_t kEncoderMotor0EncoderPinB = GPIO_NUM_19;

constexpr gpio_num_t kEncoderMotor1PositivePin = GPIO_NUM_4;  // 编码电机1正极引脚。 | Encoder motor 1 positive pin.
constexpr gpio_num_t kEncoderMotor1NegativePin = GPIO_NUM_2;  // 编码电机1负极引脚。 | Encoder motor 1 negative pin.
// 编码电机1编码器A相引脚。 | Encoder motor 1 encoder phase A pin.
constexpr gpio_num_t kEncoderMotor1EncoderPinA = GPIO_NUM_5;
// 编码电机1编码器B相引脚。 | Encoder motor 1 encoder phase B pin.
constexpr gpio_num_t kEncoderMotor1EncoderPinB = GPIO_NUM_23;

constexpr gpio_num_t kEncoderMotor2PositivePin = GPIO_NUM_17;  // 编码电机2正极引脚。 | Encoder motor 2 positive pin.
constexpr gpio_num_t kEncoderMotor2NegativePin = GPIO_NUM_12;  // 编码电机2负极引脚。 | Encoder motor 2 negative pin.
// 编码电机2编码器A相引脚。 | Encoder motor 2 encoder phase A pin.
constexpr gpio_num_t kEncoderMotor2EncoderPinA = GPIO_NUM_35;
// 编码电机2编码器B相引脚。 | Encoder motor 2 encoder phase B pin.
constexpr gpio_num_t kEncoderMotor2EncoderPinB = GPIO_NUM_36;

constexpr gpio_num_t kEncoderMotor3PositivePin = GPIO_NUM_15;  // 编码电机3正极引脚。 | Encoder motor 3 positive pin.
constexpr gpio_num_t kEncoderMotor3NegativePin = GPIO_NUM_14;  // 编码电机3负极引脚。 | Encoder motor 3 negative pin.
// 编码电机3编码器A相引脚。 | Encoder motor 3 encoder phase A pin.
constexpr gpio_num_t kEncoderMotor3EncoderPinA = GPIO_NUM_34;
// 编码电机3编码器B相引脚。 | Encoder motor 3 encoder phase B pin.
constexpr gpio_num_t kEncoderMotor3EncoderPinB = GPIO_NUM_39;

constexpr uint32_t kEncoderPpr = 12;      // 每转脉冲数。 | Pulses per revolution.
constexpr uint32_t kReductionRatio = 90;  // 减速比。 | Reduction ratio.
// 编码电机正转时B相领先于A相。 | When the encoder motor runs forward, phase B leads phase A.
constexpr auto kEncoderPhaseRelation = em::EncoderMotor::kBPhaseLeads;

constexpr float kSpeedPidP = 1.5f;  // 速度PID比例系数。 | Speed PID proportional coefficient.
constexpr float kSpeedPidI = 1.5f;  // 速度PID积分系数。 | Speed PID integral coefficient.
constexpr float kSpeedPidD = 1.0f;  // 速度PID微分系数。 | Speed PID derivative coefficient.

constexpr uint32_t kMotorUpdateIntervalMs = 100;  // 电机更新间隔时间，单位毫秒。 | Motor update interval in milliseconds.

// 连接手柄超时时间，单位毫秒。 | Connection timeout for gamepad in milliseconds.
constexpr uint32_t kConnectionTimeoutMs = 5000;

uint32_t g_motor_last_update_time = 0;

// 四个编码电机对象。 | Four encoder motor objects.
em::EncoderMotor g_encoder_motor_0(kEncoderMotor0PositivePin, kEncoderMotor0NegativePin, kEncoderMotor0EncoderPinA,
                                   kEncoderMotor0EncoderPinB, kEncoderPpr, kReductionRatio, kEncoderPhaseRelation);
em::EncoderMotor g_encoder_motor_1(kEncoderMotor1PositivePin, kEncoderMotor1NegativePin, kEncoderMotor1EncoderPinA,
                                   kEncoderMotor1EncoderPinB, kEncoderPpr, kReductionRatio, kEncoderPhaseRelation);
em::EncoderMotor g_encoder_motor_2(kEncoderMotor2PositivePin, kEncoderMotor2NegativePin, kEncoderMotor2EncoderPinA,
                                   kEncoderMotor2EncoderPinB, kEncoderPpr, kReductionRatio, kEncoderPhaseRelation);
em::EncoderMotor g_encoder_motor_3(kEncoderMotor3PositivePin, kEncoderMotor3NegativePin, kEncoderMotor3EncoderPinA,
                                   kEncoderMotor3EncoderPinB, kEncoderPpr, kReductionRatio, kEncoderPhaseRelation);

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

void EncoderMotorInit() {
  printf("Encoder motor driver init.\n");
  g_encoder_motor_0.Init();
  g_encoder_motor_1.Init();
  g_encoder_motor_2.Init();
  g_encoder_motor_3.Init();

  g_encoder_motor_0.SetSpeedPid(kSpeedPidP, kSpeedPidI, kSpeedPidD);
  g_encoder_motor_1.SetSpeedPid(kSpeedPidP, kSpeedPidI, kSpeedPidD);
  g_encoder_motor_2.SetSpeedPid(kSpeedPidP, kSpeedPidI, kSpeedPidD);
  g_encoder_motor_3.SetSpeedPid(kSpeedPidP, kSpeedPidI, kSpeedPidD);

  g_encoder_motor_0.RunPwmDuty(0);
  g_encoder_motor_1.RunPwmDuty(0);
  g_encoder_motor_2.RunPwmDuty(0);
  g_encoder_motor_3.RunPwmDuty(0);
}
}  // namespace

void setup() {
  EncoderMotorInit();

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
      g_encoder_motor_0.Stop();
      g_encoder_motor_1.Stop();
      g_encoder_motor_2.Stop();
      g_encoder_motor_3.Stop();
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kUp) && !it.holding(Button::kDown)) {
      // 上方向键：电机正转 | Up button: move forward.
      printf("Forward.\n");
      g_encoder_motor_0.RunPwmDuty(em::Motor::kMaxPwmDuty);
      g_encoder_motor_1.RunPwmDuty(em::Motor::kMaxPwmDuty);
      g_encoder_motor_2.RunPwmDuty(em::Motor::kMaxPwmDuty);
      g_encoder_motor_3.RunPwmDuty(em::Motor::kMaxPwmDuty);
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kDown) && !it.holding(Button::kUp)) {
      // 下方向键：电机反转 | Down button: move backward.
      printf("Reverse.\n");
      g_encoder_motor_0.RunPwmDuty(-em::Motor::kMaxPwmDuty);
      g_encoder_motor_1.RunPwmDuty(-em::Motor::kMaxPwmDuty);
      g_encoder_motor_2.RunPwmDuty(-em::Motor::kMaxPwmDuty);
      g_encoder_motor_3.RunPwmDuty(-em::Motor::kMaxPwmDuty);
      g_motor_last_update_time = millis();
    } else {
      // 无输入：停止 | No input: stop.
      printf("Stop.\n");
      g_encoder_motor_0.RunPwmDuty(0);
      g_encoder_motor_1.RunPwmDuty(0);
      g_encoder_motor_2.RunPwmDuty(0);
      g_encoder_motor_3.RunPwmDuty(0);
      g_motor_last_update_time = millis();
    }
  }
}