# CodexPad-S10手柄控制示例

[English](README.md)

本仓库是**CodexPad-S10**蓝牙手柄的示例程序集合，演示如何使用手柄在不同硬件平台上控制直流电机、编码电机、舵机等外设，以及基于BLE‑UNO开发板的麦克纳姆轮小车综合示例。示例按开发板平台和功能类型分目录组织。用户可自行选择对应示例进行烧录运行。

## 示例结构

示例程序按硬件平台、功能类型和具体示例三个层级组织：

```text
├── maker_esp32                      # MAKER-ESP32 / MAKER-ESP32-PRO开发板示例
│   └── base                         
│       ├── dc_motor                 # 直流电机示例，手柄控制4个直流电机
│       ├── encoder_motor            # 编码电机示例，手柄控制4个编码电机
│       ├── servo                    # 舵机示例，手柄控制4个舵机
│       └── encoder_motor_and_servo  # 编码电机+舵机综合示例
│
├── ai_vox3                          # AI-VOX3开发板示例
│   └── base                        
│       ├── dc_motor                 # 直流电机示例，手柄控制4个直流电机
│       ├── encoder_motor            # 编码电机示例，手柄控制4个编码电机
│       ├── servo                    # 舵机示例，手柄控制4个舵机
│       └── encoder_motor_and_servo  # 编码电机+舵机综合示例
│
└── ble_uno                          # BLE-UNO开发板示例
    ├── base                         
    │   ├── dc_motor                 # 直流电机示例，手柄控制4个直流电机
    │   ├── encoder_motor            # 编码电机示例，手柄控制4个编码电机
    │   ├── servo                    # 舵机示例，手柄控制4个舵机
    │   └── encoder_motor_and_servo  # 编码电机+舵机综合示例
    └── car                      
        ├── mecanum_car              # 纯麦克纳姆轮小车示例，全向移动+速度调节+自旋转
        └── mecanum_car_with_servo        # 带舵机的麦克纳姆轮小车示例，全向移动+云台俯仰+夹子开合
```

## 使用说明

### 基础示例（base/）

基础示例（base/）演示了如何通过手柄单独控制某一类外设，覆盖直流电机、编码电机和舵机的控制演示。每个示例都针对特定的硬件方案设计，您可以根据自己使用的开发板和驱动模块，点击下方链接查看对应的详细说明：

| 硬件方案（点击查看示例说明） |
| :--- |
| [MAKER-ESP32开发板（板载电机驱动）](https://gitee.com/nulllab_1/docs_examples_gamepad_peripheral_control/blob/main/examples_description_maker_esp32_base.zh-CN.md#codexpad-s10手柄控制示例maker-esp32开发板示例说明) |
| [MAKER-ESP32-PRO开发板（板载电机驱动）](https://gitee.com/nulllab_1/docs_examples_gamepad_peripheral_control/blob/main/examples_description_maker_esp32_pro_base.zh-CN.md#codexpad-s10手柄控制示例maker-esp32-pro开发板示例说明) |
| [AI-VOX3开发板 + MD40驱动模块](https://gitee.com/nulllab_1/docs_examples_gamepad_peripheral_control/blob/main/examples_description_ai_vox3_base.zh-CN.md#codexpad-s10手柄控制示例ai-vox3开发板示例说明) |
| [BLE-UNO开发板 + MotorDriverBoard V5.2驱动板](https://gitee.com/nulllab_1/docs_examples_gamepad_peripheral_control/blob/main/examples_description_ble_uno_base.zh-CN.md#codexpad-s10手柄控制示例ble-uno开发板示例说明) |

### 麦克纳姆轮小车示例（car/）

麦克纳姆轮小车示例演示如何使用手柄遥控麦克纳姆轮小车，包含全向移动、速度调节、云台俯仰和夹子开合等完整功能。**目前仅限BLE-UNO平台**。

| 硬件方案（点击查看示例说明） |
| :--- |
| [BLE-UNO开发板 + MotorDriverBoard V5.2驱动板](https://gitee.com/nulllab_1/docs_examples_gamepad_peripheral_control/blob/main/examples_description_ble_uno_car.zh-CN.md#codexpad-s10手柄控制麦克纳姆轮小车ble-uno开发板示例说明) |
