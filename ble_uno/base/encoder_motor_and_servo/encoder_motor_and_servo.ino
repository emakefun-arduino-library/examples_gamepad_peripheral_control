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
 *
 * @note Debugging Notes:
 *       - The hardware serial (D0/D1) is occupied by the Bluetooth module and cannot be used for debug output.
 *       - The SoftwareSerial soft serial port conflicts with the Emakefun Motor Driver library and cannot be used
 *         simultaneously.
 *       - Therefore, this example does not include any serial debug output. For debugging, it is recommended to use LED
 *         indicators or buzzers instead.
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
 *
 * @note 调试说明：
 *       - 硬件串口（D0/D1）已被蓝牙模块占用，无法用于调试输出。
 *       - SoftwareSerial软串口与Emakefun Motor Driver库存在冲突，无法同时使用。
 *       - 因此本示例不包含任何串口调试输出，如需调试建议使用LED指示灯或蜂鸣器等方式。
 */

#include "Emakefun_MotorDriver.h"
#include "gamepad_codec_decoder.h"

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
// Replace with your target device's Bluetooth device address.
// 替换为目标设备的 Bluetooth device address。
const String kBluetoothDeviceAddress = "16:00:00:00:03:27";

// The serial port baud rate for communication with the Bluetooth module (please refer to the module data manual, replace with
// the corresponding baud rate for the module).
// 与蓝牙模块通信的串口波特率（请查阅模块数据手册，替换为模块对应的波特率）。
constexpr uint32_t kBluetoothModuleSerialBaudRate = 115200;

constexpr uint8_t kServo1Pin = 1;  // 舵机1位置 S1。 | Servo 1 S1.
constexpr uint8_t kServo2Pin = 2;  // 舵机2位置 S2。 | Servo 2 S2.
constexpr uint8_t kServo3Pin = 3;  // 舵机3位置 S3。 | Servo 3 S3.
constexpr uint8_t kServo4Pin = 4;  // 舵机4位置 S4。 | Servo 4 S4.

constexpr uint8_t kMotorDriverI2CAddress = 0x60;  // 驱动板I2C地址。 | Driver board I2C address.
constexpr uint8_t kPwmFrequency = 50;             // PWM频率，单位 Hz。 | PWM frequency in Hz.

constexpr uint8_t kMotorRunSpeed = 255;           // 电机运行速度 (0~255) | Motor running speed (0~255).
constexpr uint32_t kMotorUpdateIntervalMs = 100;  // 更新间隔（毫秒） | Update interval in milliseconds.

constexpr uint8_t kServoAngleMax = 180;          // 舵机最大角度。 | Maximum angle of the servo.
constexpr uint8_t kServoRunSpeed = 10;           // 舵机运转速度。 | Servo running speed.
constexpr uint8_t kServoUpdateIntervalMs = 100;  // 舵机更新间隔（毫秒）。 | Servo update interval in milliseconds.

uint32_t g_motor_last_update_time = 0;
uint32_t g_servo_last_update_time = 0;

// 驱动板实例。 | Driver board instance.
Emakefun_MotorDriver g_motor_driver(kMotorDriverI2CAddress);

// 四个编码电机对象。 | Four encoder motor objects.
Emakefun_EncoderMotor *g_encoder_motor_0 = g_motor_driver.getEncoderMotor(E1);
Emakefun_EncoderMotor *g_encoder_motor_1 = g_motor_driver.getEncoderMotor(E2);
Emakefun_EncoderMotor *g_encoder_motor_2 = g_motor_driver.getEncoderMotor(E3);
Emakefun_EncoderMotor *g_encoder_motor_3 = g_motor_driver.getEncoderMotor(E4);

// 四个舵机对象。 | Four servo objects.
Emakefun_Servo *g_servo_1 = g_motor_driver.getServo(kServo1Pin);
Emakefun_Servo *g_servo_2 = g_motor_driver.getServo(kServo2Pin);
Emakefun_Servo *g_servo_3 = g_motor_driver.getServo(kServo3Pin);
Emakefun_Servo *g_servo_4 = g_motor_driver.getServo(kServo4Pin);

// The serial instance passed in by the decoder here is the same as the one passed in Connect().
// 此处解码器传入的串口实例，与 Connect() 中传入的为同一个串口实例。
gamepad::codec::Decoder g_decoder(Serial);

/**
 * @~English
 * @brief Send AT commands to the Bluetooth module to establish a BLE connection.
 * @details Executes a sequence of AT commands to configure the Bluetooth module as master and connect to the target device
 *          at the specified MAC address.
 *          Command order: AT+DISCON → AT+RESET → AT+ECHO=0 → AT+ROLE=0 → AT+AUTOCON=0 → AT+CON=<mac>.
 * @param[in] bluetooth_stream Stream connected to the Bluetooth module.
 *                             Note: This must be the same Stream instance that is passed to the CodexPadFrameDecoder
 *                             (e.g., Serial), as the decoder reads incoming data from this same serial port.
 * @param[in] bluetooth_device_address MAC address of the target device in format "XX:XX:XX:XX:XX:XX".
 */
/**
 * @~Chinese
 * @brief 向蓝牙模块发送 AT 指令以建立 BLE 连接。
 * @details 依次执行 AT 指令序列，将蓝牙模块配置为主机模式并连接到指定 MAC 地址的目标设备。
 *          指令顺序：AT+DISCON → AT+RESET → AT+ECHO=0 → AT+ROLE=0 → AT+AUTOCON=0 → AT+CON=<mac>。
 * @param[in] bluetooth_stream 连接蓝牙模块的 Stream 对象。
 *                             注意：此对象必须与传入 CodexPadFrameDecoder 的 Stream 是同一个实例（如 Serial），
 *                             因为解码器正是从这同一个串口读取手柄发来的数据。
 * @param[in] bluetooth_device_address 目标设备的 MAC 地址，格式为 "XX:XX:XX:XX:XX:XX"。
 */
void Connect(Stream &bluetooth_stream, const String &bluetooth_device_address) {
  if (bluetooth_device_address.length() != 17 || bluetooth_device_address[2] != ':' || bluetooth_device_address[5] != ':' ||
      bluetooth_device_address[8] != ':' || bluetooth_device_address[11] != ':' || bluetooth_device_address[14] != ':') {
    while (true);
  }

  // The module may be in a connected state. Send the disconnection command first to ensure the module is in an unconnected
  // state.
  // 模块可能处于连接状态，先发送断开指令，确保模块是未连接状态。
  bluetooth_stream.println("AT+DISCON");
  delay(100);

  // Software reset BLE chip, clear all pairing and configuration data.
  // 软件复位蓝牙芯片，清除所有配对和配置数据。
  bluetooth_stream.println("AT+RESET");
  delay(100);

  // Close AT information echo.
  // 关闭AT信息回显。
  bluetooth_stream.println("AT+ECHO=0");
  delay(100);

  // Set the module to host mode so that it can actively connect to the BLE of the slave device.
  // 设置模块为主机模式，使其能够主动连接从机蓝牙。
  bluetooth_stream.println("AT+ROLE=0");
  delay(100);

  // Disable the module's automatic Bluetooth connection mode.
  // 关闭模块的蓝牙自动连接模式。
  bluetooth_stream.println("AT+AUTOCON=0");
  delay(100);

  // Initiate BLE connection with the slave using the specified MAC address.
  // 使用指定的MAC地址发起与从机蓝牙连接。
  bluetooth_stream.print("AT+CON=");
  bluetooth_stream.println(bluetooth_device_address);
  delay(100);
}

void SetServosAngle(const uint8_t angle) {
  if (angle <= kServoAngleMax) {
    g_servo_1->writeServo(angle, kServoRunSpeed);
    g_servo_2->writeServo(angle, kServoRunSpeed);
    g_servo_3->writeServo(angle, kServoRunSpeed);
    g_servo_4->writeServo(angle, kServoRunSpeed);
  }
}
}  // namespace

void setup() {
  g_motor_driver.begin(kPwmFrequency);

  g_encoder_motor_0->run(BRAKE);
  g_encoder_motor_1->run(BRAKE);
  g_encoder_motor_2->run(BRAKE);
  g_encoder_motor_3->run(BRAKE);

  SetServosAngle(0);

  Serial.begin(kBluetoothModuleSerialBaudRate);
  Connect(Serial, kBluetoothDeviceAddress);
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
  const Tracker &it = g_decoder.Update();
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

  if (millis() - g_motor_last_update_time >= kMotorUpdateIntervalMs) {
    if (it.holding(Button::kCrossA)) {
      // Cross按钮：刹车 | Cross button: brake.
      g_encoder_motor_0->run(BRAKE);
      g_encoder_motor_1->run(BRAKE);
      g_encoder_motor_2->run(BRAKE);
      g_encoder_motor_3->run(BRAKE);
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kUp) && !it.holding(Button::kDown)) {
      // 上方向键：电机正转 | Up button: move forward.
      g_encoder_motor_0->setSpeed(kMotorRunSpeed);
      g_encoder_motor_0->run(FORWARD);
      g_encoder_motor_1->setSpeed(kMotorRunSpeed);
      g_encoder_motor_1->run(FORWARD);
      g_encoder_motor_2->setSpeed(kMotorRunSpeed);
      g_encoder_motor_2->run(FORWARD);
      g_encoder_motor_3->setSpeed(kMotorRunSpeed);
      g_encoder_motor_3->run(FORWARD);
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kDown) && !it.holding(Button::kUp)) {
      // 下方向键：电机反转 | Down button: move backward.
      g_encoder_motor_0->setSpeed(kMotorRunSpeed);
      g_encoder_motor_0->run(BACKWARD);
      g_encoder_motor_1->setSpeed(kMotorRunSpeed);
      g_encoder_motor_1->run(BACKWARD);
      g_encoder_motor_2->setSpeed(kMotorRunSpeed);
      g_encoder_motor_2->run(BACKWARD);
      g_encoder_motor_3->setSpeed(kMotorRunSpeed);
      g_encoder_motor_3->run(BACKWARD);
      g_motor_last_update_time = millis();
    } else {
      // 无输入：刹车 | No input: brake.
      g_encoder_motor_0->run(BRAKE);
      g_encoder_motor_1->run(BRAKE);
      g_encoder_motor_2->run(BRAKE);
      g_encoder_motor_3->run(BRAKE);
      g_motor_last_update_time = millis();
    }
  }

  if (millis() - g_servo_last_update_time >= kServoUpdateIntervalMs) {
    const bool square = it.pressed(Button::kSquareX);
    const bool triangle = it.pressed(Button::kTriangleY);
    const bool circle = it.pressed(Button::kCircleB);

    if (square && !triangle && !circle) {
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