
/**
 * @~English
 * @brief Control a Mecanum wheel car (with pan/tilt and claw servos) using the CodexPad-S10 gamepad.
 * @details Control the omnidirectional movement, speed adjustment, pan/tilt and claw of a Mecanum wheel car via gamepad
 * buttons.
 *          - Up button (holding): Move forward.
 *          - Down button (holding): Move backward.
 *          - Left button (holding): Strafe left.
 *          - Right button (holding): Strafe right.
 *          - Up + Right (holding): Move forward-right 45°.
 *          - Down + Left (holding): Move backward-left 45°.
 *          - Up + Left (holding): Move forward-left 45°.
 *          - Down + Right (holding): Move backward-right 45°.
 *          - Square button (holding): Rotate counterclockwise.
 *          - Circle button (holding): Rotate clockwise.
 *          - L1 button (pressed): Increase motor speed (+5 per press, range 0-255).
 *          - R1 button (pressed): Decrease motor speed (-5 per press, range 0-255).
 *          - Triangle button (holding): Pan/tilt up (angle gradually increases).
 *          - Cross button (holding): Pan/tilt down (angle gradually decreases).
 *          - L2 button (holding): Open claw (angle gradually increases).
 *          - R2 button (holding): Close claw (angle gradually decreases).
 *          - No button input: All motors brake.
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
 * @brief 使用CodexPad-S10手柄控制麦克纳姆轮小车（含云台和夹子舵机）。
 * @details 通过手柄按钮控制麦克纳姆轮小车的全向移动、速度调节、云台俯仰和夹子开合。
 *          - 上方向按钮（按住）：前进。
 *          - 下方向按钮（按住）：后退。
 *          - 左方向按钮（按住）：左平移。
 *          - 右方向按钮（按住）：右平移。
 *          - 上+右（按住）：右前方 45° 移动。
 *          - 下+左（按住）：左后方 45° 移动。
 *          - 上+左（按住）：左前方 45° 移动。
 *          - 下+右（按住）：右后方 45° 移动。
 *          - Square 按钮（按住）：原地逆时针旋转。
 *          - Circle 按钮（按住）：原地顺时针旋转。
 *          - L1 按钮（按下）：增加电机速度（每按一次 +5，范围 0-255）。
 *          - R1 按钮（按下）：降低电机速度（每按一次 -5，范围 0-255）。
 *          - Triangle 按钮（按住）：云台上仰（角度逐渐增大）。
 *          - Cross 按钮（按住）：云台下俯（角度逐渐减小）。
 *          - L2 按钮（按住）：夹子张开（角度逐渐增大）。
 *          - R2 按钮（按住）：夹子闭合（角度逐渐减小）。
 *          - 无按钮输入时，所有电机刹车。
 *
 * @note 调试说明：
 *       - 硬件串口（D0/D1）已被蓝牙模块占用，无法用于调试输出。
 *       - SoftwareSerial 软串口与 Emakefun Motor Driver 库存在冲突，无法同时使用。
 *       - 因此本示例不包含任何串口调试输出，如需调试建议使用 LED 指示灯或蜂鸣器等方式。
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

constexpr uint8_t kPanTiltServoPin = 2;  // 云台俯仰舵机引脚。 | Pan-tilt servo pin.
constexpr uint8_t kClawServoPin = 1;     // 云台夹子舵机引脚。 | Claw servo pin.

constexpr uint8_t kLeftFrontMotorPort = M1;   // 左前轮电机口M1。 | Left front motor port M1.
constexpr uint8_t kLeftBackMotorPort = M2;    // 左后轮电机口M2。 | Left back motor port M2.
constexpr uint8_t kRightFrontMotorPort = M3;  // 右前轮电机口M3。 | Right front motor port M3.
constexpr uint8_t kRightBackMotorPort = M4;   // 右后轮电机口M4。 | Right back motor port M4.

constexpr uint8_t kMotorDriverI2CAddress = 0x60;  // 驱动板 I2C 地址。 | Driver board I2C address.
constexpr uint8_t kPwmFrequency = 50;             // PWM 频率，单位 Hz。 | PWM frequency in Hz.

constexpr uint8_t kMotorSpeedMin = 0;             // 电机最小速度值。 | Motor minimum speed value.
constexpr uint8_t kMotorSpeedMax = 255;           // 电机最大速度值。 | Motor maximum speed value.
constexpr uint8_t kMotorSpeedStep = 5;            // 电机速度每次调节步长。 | Motor speed adjustment step per press.
constexpr uint32_t kMotorUpdateIntervalMs = 100;  // 电机更新间隔（毫秒） | Update interval in milliseconds.

constexpr uint8_t kPanTiltServoAngleMin = 0;    // 云台舵机最小角度。 | Pan-tilt servo minimum angle.
constexpr uint8_t kPanTiltServoAngleMax = 150;  // 云台舵机最大角度。 | Pan-tilt servo maximum angle.
constexpr uint8_t kPanTiltServoAngleStep = 2;   // 云台舵机角度调节步长。 | Pan-tilt servo angle adjustment step.

constexpr uint8_t kClawServoAngleMin = 10;   // 夹子舵机最小角度。 | Claw servo minimum angle.
constexpr uint8_t kClawServoAngleMax = 100;  // 夹子舵机最大角度。 | Claw servo maximum angle.
constexpr uint8_t kClawServoAngleStep = 2;   // 夹子舵机角度调节步长。 | Claw servo angle adjustment step.

constexpr uint8_t kServoDelayMs = 20;   // 舵机控制延时，单位毫秒。 | Servo control delay in milliseconds.
constexpr uint8_t kServoRunSpeed = 10;  // 舵机运转速度。 | Servo running speed.

uint8_t g_motor_current_speed = 100;
uint8_t g_pan_tilt_servo_current_angle = 90;
uint8_t g_claw_servo_current_angle = 30;

uint32_t g_pan_tilt_servo_last_update_time = 0;
uint32_t g_claw_servo_last_update_time = 0;
uint32_t g_motor_last_update_time = 0;

Emakefun_MotorDriver g_motor_driver = Emakefun_MotorDriver(kMotorDriverI2CAddress);

// 两个舵机对象。 | Two servo objects.
Emakefun_Servo *g_pan_tilt_servo = g_motor_driver.getServo(kPanTiltServoPin);
Emakefun_Servo *g_claw_servo = g_motor_driver.getServo(kClawServoPin);

// 四个直流电机对象。 | Four DC motor objects.
Emakefun_DCMotor *g_left_front_motor = g_motor_driver.getMotor(kLeftFrontMotorPort);
Emakefun_DCMotor *g_left_back_motor = g_motor_driver.getMotor(kLeftBackMotorPort);
Emakefun_DCMotor *g_right_front_motor = g_motor_driver.getMotor(kRightFrontMotorPort);
Emakefun_DCMotor *g_right_back_motor = g_motor_driver.getMotor(kRightBackMotorPort);

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
}  // namespace

void setup() {
  g_motor_driver.begin(kPwmFrequency);

  g_pan_tilt_servo->writeServo(g_pan_tilt_servo_current_angle, kServoRunSpeed);
  g_claw_servo->writeServo(g_claw_servo_current_angle, kServoRunSpeed);

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

  if (it.pressed(Button::kL1)) {
    // 增加电机速度（按下L1按键）。 | Increase motor speed (pressed L1 button).
    g_motor_current_speed = constrain(g_motor_current_speed + kMotorSpeedStep, kMotorSpeedMin, kMotorSpeedMax);
  } else if (it.pressed(Button::kR1)) {
    // 降低电机速度（按下R1按键）。 | Decrease motor speed (pressed R1 button).
    g_motor_current_speed = constrain(g_motor_current_speed - kMotorSpeedStep, kMotorSpeedMin, kMotorSpeedMax);
  }

  if (millis() - g_motor_last_update_time >= kMotorUpdateIntervalMs) {
    const bool up = it.holding(Button::kUp);
    const bool down = it.holding(Button::kDown);
    const bool left = it.holding(Button::kLeft);
    const bool right = it.holding(Button::kRight);

    if (up && !left && !right) {
      // 前进（单独按住上方向按钮）。 | Move forward (hold Up button only).
      g_left_front_motor->setSpeed(g_motor_current_speed);
      g_left_front_motor->run(BACKWARD);
      g_left_back_motor->setSpeed(g_motor_current_speed);
      g_left_back_motor->run(BACKWARD);
      g_right_front_motor->setSpeed(g_motor_current_speed);
      g_right_front_motor->run(FORWARD);
      g_right_back_motor->setSpeed(g_motor_current_speed);
      g_right_back_motor->run(FORWARD);
      g_motor_last_update_time = millis();
    } else if (down && !left && !right) {
      // 后退（单独按住下方向按钮）。 | Move backward (hold Down button only).
      g_left_front_motor->setSpeed(g_motor_current_speed);
      g_left_front_motor->run(FORWARD);
      g_left_back_motor->setSpeed(g_motor_current_speed);
      g_left_back_motor->run(FORWARD);
      g_right_front_motor->setSpeed(g_motor_current_speed);
      g_right_front_motor->run(BACKWARD);
      g_right_back_motor->setSpeed(g_motor_current_speed);
      g_right_back_motor->run(BACKWARD);
      g_motor_last_update_time = millis();
    } else if (left && !up && !down) {
      // 左平移（单独按住左方向按钮）。 | Strafe left (hold Left button only).
      g_left_front_motor->setSpeed(g_motor_current_speed);
      g_left_front_motor->run(FORWARD);
      g_left_back_motor->setSpeed(g_motor_current_speed);
      g_left_back_motor->run(BACKWARD);
      g_right_front_motor->setSpeed(g_motor_current_speed);
      g_right_front_motor->run(FORWARD);
      g_right_back_motor->setSpeed(g_motor_current_speed);
      g_right_back_motor->run(BACKWARD);
      g_motor_last_update_time = millis();
    } else if (right && !up && !down) {
      // 右平移（单独按住右方向按钮）。 | Strafe right (hold Right button only).
      g_left_front_motor->setSpeed(g_motor_current_speed);
      g_left_front_motor->run(BACKWARD);
      g_left_back_motor->setSpeed(g_motor_current_speed);
      g_left_back_motor->run(FORWARD);
      g_right_front_motor->setSpeed(g_motor_current_speed);
      g_right_front_motor->run(BACKWARD);
      g_right_back_motor->setSpeed(g_motor_current_speed);
      g_right_back_motor->run(FORWARD);
      g_motor_last_update_time = millis();
    } else if (up && right) {
      // 右前方45°移动（同时按住上方向和右方向按钮）。 | Move forward-right 45° (hold Up + Right button).
      g_left_front_motor->setSpeed(g_motor_current_speed);
      g_left_front_motor->run(BACKWARD);
      g_left_back_motor->run(BRAKE);
      g_right_front_motor->run(BRAKE);
      g_right_back_motor->setSpeed(g_motor_current_speed);
      g_right_back_motor->run(FORWARD);
      g_motor_last_update_time = millis();
    } else if (left && down) {
      // 左后方45°移动（同时按住左方向和下方向按钮）。 | Move backward-left 45° (hold Left + Down button).
      g_left_front_motor->setSpeed(g_motor_current_speed);
      g_left_front_motor->run(FORWARD);
      g_left_back_motor->run(BRAKE);
      g_right_front_motor->run(BRAKE);
      g_right_back_motor->setSpeed(g_motor_current_speed);
      g_right_back_motor->run(BACKWARD);
      g_motor_last_update_time = millis();
    } else if (up && left) {
      // 左前方45°移动（同时按住上方向和左方向按钮）。 | Move forward-left 45° (hold Up + Left button).
      g_left_front_motor->run(BRAKE);
      g_left_back_motor->setSpeed(g_motor_current_speed);
      g_left_back_motor->run(BACKWARD);
      g_right_front_motor->setSpeed(g_motor_current_speed);
      g_right_front_motor->run(FORWARD);
      g_right_back_motor->run(BRAKE);
      g_motor_last_update_time = millis();
    } else if (right && down) {
      // 右后方45°移动（同时按住右方向和下方向按钮）。 | Move backward-right 45° (hold Right + Down button).
      g_left_front_motor->run(BRAKE);
      g_left_back_motor->setSpeed(g_motor_current_speed);
      g_left_back_motor->run(FORWARD);
      g_right_front_motor->setSpeed(g_motor_current_speed);
      g_right_front_motor->run(BACKWARD);
      g_right_back_motor->run(BRAKE);
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kSquareX)) {
      // 原地逆时针旋转（按住Square按钮）。 | Rotate counterclockwise (hold Square button).
      g_left_front_motor->setSpeed(g_motor_current_speed);
      g_left_front_motor->run(FORWARD);
      g_left_back_motor->setSpeed(g_motor_current_speed);
      g_left_back_motor->run(FORWARD);
      g_right_front_motor->setSpeed(g_motor_current_speed);
      g_right_front_motor->run(FORWARD);
      g_right_back_motor->setSpeed(g_motor_current_speed);
      g_right_back_motor->run(FORWARD);
      g_motor_last_update_time = millis();
    } else if (it.holding(Button::kCircleB)) {
      // 原地顺时针旋转（按住Circle按钮）。 | Rotate clockwise (hold Circle button).
      g_left_front_motor->setSpeed(g_motor_current_speed);
      g_left_front_motor->run(BACKWARD);
      g_left_back_motor->setSpeed(g_motor_current_speed);
      g_left_back_motor->run(BACKWARD);
      g_right_front_motor->setSpeed(g_motor_current_speed);
      g_right_front_motor->run(BACKWARD);
      g_right_back_motor->setSpeed(g_motor_current_speed);
      g_right_back_motor->run(BACKWARD);
      g_motor_last_update_time = millis();
    } else {
      // 无方向输入：刹车。 | No direction input: brake.
      g_left_front_motor->run(BRAKE);
      g_right_front_motor->run(BRAKE);
      g_left_back_motor->run(BRAKE);
      g_right_back_motor->run(BRAKE);
      g_motor_last_update_time = millis();
    }
  }

  if (it.holding(Button::kTriangleY) && (millis() - g_pan_tilt_servo_last_update_time >= kServoDelayMs)) {
    // 云台上仰（按住Triangle按钮）。 | Pan-tilt up (hold Triangle button).
    g_pan_tilt_servo_current_angle =
        constrain(g_pan_tilt_servo_current_angle + kPanTiltServoAngleStep, kPanTiltServoAngleMin, kPanTiltServoAngleMax);
    g_pan_tilt_servo->writeServo(g_pan_tilt_servo_current_angle, kServoRunSpeed);
    g_pan_tilt_servo_last_update_time = millis();
  } else if (it.holding(Button::kCrossA) && (millis() - g_pan_tilt_servo_last_update_time >= kServoDelayMs)) {
    // 云台下俯（按住Cross按钮）。 | Pan-tilt down (hold Cross button).
    g_pan_tilt_servo_current_angle =
        constrain(g_pan_tilt_servo_current_angle - kPanTiltServoAngleStep, kPanTiltServoAngleMin, kPanTiltServoAngleMax);
    g_pan_tilt_servo->writeServo(g_pan_tilt_servo_current_angle, kServoRunSpeed);
    g_pan_tilt_servo_last_update_time = millis();
  }

  if (it.holding(Button::kL2) && (millis() - g_claw_servo_last_update_time >= kServoDelayMs)) {
    // 夹子张开（按住L2按钮）。 | Open claw (hold L2 button).
    g_claw_servo_current_angle =
        constrain(g_claw_servo_current_angle + kClawServoAngleStep, kClawServoAngleMin, kClawServoAngleMax);
    g_claw_servo->writeServo(g_claw_servo_current_angle, kServoRunSpeed);
    g_claw_servo_last_update_time = millis();
  } else if (it.holding(Button::kR2) && (millis() - g_claw_servo_last_update_time >= kServoDelayMs)) {
    // 夹子闭合（按住R2按钮）。 | Close claw (hold R2 button).
    g_claw_servo_current_angle =
        constrain(g_claw_servo_current_angle - kClawServoAngleStep, kClawServoAngleMin, kClawServoAngleMax);
    g_claw_servo->writeServo(g_claw_servo_current_angle, kServoRunSpeed);
    g_claw_servo_last_update_time = millis();
  }
}