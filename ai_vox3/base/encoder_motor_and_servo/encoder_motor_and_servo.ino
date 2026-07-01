/**
 * @~English
 * @brief Control 4 encoder motors and 4 servos using CodexPad-S10 gamepad.
 * @details Control 4 encoder motors and 4 servos with gamepad buttons:
 *          - Up button (holding): All 4 encoder motors run forward (PWM duty 1023).
 *          - Down button (holding): All 4 encoder motors run backward (PWM duty -1023).
 *          - Cross button (holding): All 4 encoder motors brake (call Stop method), brake has the highest priority.
 *          - Square button (pressed): Set all 4 servos to 0 degrees.
 *          - Triangle button (pressed): Set all 4 servos to 90 degrees.
 *          - Circle button (pressed): Set all 4 servos to 180 degrees.
 */
/**
 * @~Chinese
 * @brief 使用CodexPad-S10手柄控制4个编码电机和4个舵机。
 * @details 通过手柄按钮控制编码4个电机和4个舵机：
 *          - 上方向按钮（按住）：4个编码电机同时正转（PWM占空比 1023）。
 *          - 下方向按钮（按住）：4个编码电机同时反转（PWM占空比 -1023）。
 *          - Cross按钮（按住）：4个编码电机刹车（调用Stop方法），刹车优先级最高。
 *          - Square按钮（按下）：设置4个舵机角度为 0 度。
 *          - Triangle按钮（按下）：设置4个舵机角度为 90 度。
 *          - Circle按钮（按下）：设置4个舵机角度为 180 度。
 */

#include "codex_pad.h"
#include "md40.h"
#include "servo.h"

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

constexpr gpio_num_t kServo0Pin = GPIO_NUM_42;  // 舵机0引脚。 | Servo 0 pin.
constexpr gpio_num_t kServo1Pin = GPIO_NUM_43;  // 舵机1引脚。 | Servo 1 pin.
constexpr gpio_num_t kServo2Pin = GPIO_NUM_44;  // 舵机2引脚。 | Servo 2 pin.
constexpr gpio_num_t kServo3Pin = GPIO_NUM_48;  // 舵机3引脚。 | Servo 3 pin.

constexpr uint16_t kEncoderPpr = 12;      // 每转脉冲数。 | Pulses per revolution.
constexpr uint16_t kReductionRatio = 90;  // 减速比。 | Reduction ratio.
// 编码电机正转时A相领先于B相。 | When the encoder motor runs forward, phase A leads phase B.
constexpr auto kEncoderPhaseRelation = em::Md40::Motor::PhaseRelation::kAPhaseLeads;

constexpr float kSpeedPidP = 1.5f;      // 速度PID比例系数。 | Speed PID proportional coefficient.
constexpr float kSpeedPidI = 1.5f;      // 速度PID积分系数。 | Speed PID integral coefficient.
constexpr float kSpeedPidD = 1.0f;      // 速度PID微分系数。 | Speed PID derivative coefficient.
constexpr float kPositionPidP = 10.0f;  // 位置PID比例系数。 | Position PID proportional coefficient.
constexpr float kPositionPidI = 1.0f;   // 位置PID积分系数。 | Position PID integral coefficient.
constexpr float kPositionPidD = 1.0f;   // 位置PID微分系数。 | Position PID derivative coefficient.

constexpr uint16_t kMotorRunPwmDuty = 1023;       // 电机运行时的PWM占空比。 | PWM duty when motor is running.
constexpr uint32_t kMotorUpdateIntervalMs = 100;  // 电机更新间隔时间，单位毫秒。 | Motor update interval in milliseconds.

constexpr uint8_t kServoAngleMax = 180;           // 舵机最大角度。 | Servo maximum angle.
constexpr uint16_t kServoPulseWidthUsMin = 500;   // 最小脉宽，单位微秒。 | Minimum pulse width in microseconds
constexpr uint16_t kServoPulseWidthUsMax = 2500;  // 最大脉宽，单位微秒。 | Maximum pulse width in microseconds
constexpr uint8_t kServoUpdateIntervalMs = 100;   // 舵机更新间隔，单位毫秒。 | Servo update interval in milliseconds.

// 连接手柄超时时间，单位毫秒。 | Connection timeout for gamepad in milliseconds.
constexpr uint32_t kConnectionTimeoutMs = 5000;

uint32_t g_servo_last_update_time = 0;
uint32_t g_motor_last_update_time = 0;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

// 4个舵机对象。 | Servo objects.
em::Servo g_servo_0(kServo0Pin, 0, kServoAngleMax, kServoPulseWidthUsMin, kServoPulseWidthUsMax);
em::Servo g_servo_1(kServo1Pin, 0, kServoAngleMax, kServoPulseWidthUsMin, kServoPulseWidthUsMax);
em::Servo g_servo_2(kServo2Pin, 0, kServoAngleMax, kServoPulseWidthUsMin, kServoPulseWidthUsMax);
em::Servo g_servo_3(kServo3Pin, 0, kServoAngleMax, kServoPulseWidthUsMin, kServoPulseWidthUsMax);

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
  printf("Md40 init, encoder motor mode.\n");
  g_md40.Init();

  // 设置Md40运行模式为编码电机模式，并初始停止。 | Set Md40 to run in encoder motor mode and initially stop.
  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    g_md40[i].SetEncoderMode(kEncoderPpr, kReductionRatio, kEncoderPhaseRelation);
    g_md40[i].set_speed_pid_p(kSpeedPidP);
    g_md40[i].set_speed_pid_i(kSpeedPidI);
    g_md40[i].set_speed_pid_d(kSpeedPidD);
    g_md40[i].set_position_pid_p(kPositionPidP);
    g_md40[i].set_position_pid_i(kPositionPidI);
    g_md40[i].set_position_pid_d(kPositionPidD);

    g_md40[i].RunPwmDuty(0);
  }
}

void ServosInit() {
  printf("Servos init.\n");
  g_servo_0.Init();
  g_servo_1.Init();
  g_servo_2.Init();
  g_servo_3.Init();

  g_servo_0.Write(0);
  g_servo_1.Write(0);
  g_servo_2.Write(0);
  g_servo_3.Write(0);
}  // namespace

void SetServosAngle(const uint8_t angle) {
  if (angle > kServoAngleMax) {
    printf("Invalid servo angle: %u. Must be between 0 and %u.\n", angle, kServoAngleMax);
    return;
  }
  printf("Set all servos to %u degrees.\n", angle);
  g_servo_0.Write(angle);
  g_servo_1.Write(angle);
  g_servo_2.Write(angle);
  g_servo_3.Write(angle);
}
}  // namespace

void setup() {
  Wire.begin(kI2cPinSda, kI2cPinScl);

  Md40Init();
  ServosInit();

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

  if (millis() - g_servo_last_update_time >= kServoUpdateIntervalMs) {
    const bool square = it.pressed(Button::kSquareX);
    const bool triangle = it.pressed(Button::kTriangleY);
    const bool circle = it.pressed(Button::kCircleB);

    if (square && !circle && !triangle) {
      // Square按钮：设置舵机角度为0度 | Square button: set servo angle to 0 degrees.
      SetServosAngle(0);
      g_servo_last_update_time = millis();
    } else if (triangle && !square && !circle) {
      // Triangle按钮：设置舵机角度为90度 | Triangle button: set servo angle to 90 degrees.
      SetServosAngle(90);
      g_servo_last_update_time = millis();
    } else if (circle && !square && !triangle) {
      // Circle按钮：设置舵机角度为180度 | Circle button: set servo angle to 180 degrees.
      SetServosAngle(180);
      g_servo_last_update_time = millis();
    }
  }
}