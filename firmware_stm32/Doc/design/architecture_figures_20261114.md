# 当前控制系统架构图说明

更新：2026-11-14。两张新图依据默认正式构建与当前源码绘制，不修改固件。

## 图文件

| 图 | PNG | 可编辑矢量图 |
| --- | --- | --- |
| 五分区分层架构（保留原图布局） | [PNG](../../../figures/architecture/control_system_five_columns_20261114.png) | [SVG](../../../figures/architecture/control_system_five_columns_20261114.svg) |
| 控制系统总体架构 | [PNG](../../../figures/architecture/control_system_current_20261114.png) | [SVG](../../../figures/architecture/control_system_current_20261114.svg) |
| 软件分层架构 | [PNG](../../../figures/architecture/software_architecture_current_20261114.png) | [SVG](../../../figures/architecture/software_architecture_current_20261114.svg) |

总体图用于描述输入、通信、控制器、执行器与反馈关系；软件图用于描述程序分层。图中箭头表示主要信息流或调用依赖，不表示每个模块均具有独立线程。

## 与旧图的主要区别

1. 删除当前未使用的独立 BLDC 功率级、AS5600、MCU FOC、alg_pid 与 alg_voltage_foc。HX8 承担内部位置闭环，STM32 负责目标生成、限幅、安全检查和反馈监督，并有有限位置微调。
2. 图像经过模拟图传、接收与视频采集进入电脑。电脑完成识别与标定映射，再经 HC13 发送角度、速度、置信度等目标信息；HC13 不传视频。正式路径有低频接收 ACK，但 ACK 不代表运动完成。
3. ELRS 接收器通过 USART1/CRSF 接入；HX8 经 USART2、UC-01 通信；夹爪由 TIM2_CH1/PA0 输出 PWM。夹爪没有实际位置或夹持反馈，SERVO2 默认关闭。
4. VL53L1X 经 I2C1 接入，由独立任务采样并发布快照。当前正式控制链没有根据该快照完成完整的抓取闭环，不把“测距有效”画成“抓取成功”。
5. TaskRc、TaskVision、TaskArbiter、TaskManipulator 和 TaskMotion 是 ControlTask 内的顺序步骤。独立 RTOS 任务为 ControlTask、DebugTask、TofTask 和 defaultTask。20 ms 是控制任务调度周期，不是舵机内部闭环周期，也不是实测动作延迟。
6. DataHub 是 ControlTask 单写、多读的观测快照，不是实时控制命令在各层间流转的共享总线。测距快照独立发布。
7. JY61P、HC13 V2 采用另一种实验采集构建。JY61P 与 ELRS 复用 USART1，因此不画成同时接入。V2 与正式控制的协议、状态机及停止行为不能混用。

## 实现与实验边界

- 默认 AUTO 为有效视觉目标持续跟随；进入 AUTO 开爪，之后由 CH4 操作夹爪。三帧自动闭爪选项默认关闭。图中的“自动模式”不表示已实测全自动抓取成功。
- 电脑视觉和上位机源码位于外部项目；本仓库可确认 MCU 的目标接收接口与执行路径，不能确认外部算法的具体模型与实际性能。
- 主供电、飞控姿态控制及无人机航迹不在本图描述的机械臂控制程序范围内；未根据历史方案推断与飞控存在控制耦合。
- 图中的传感器接口表示程序支持与当前文档定义，不替代对当前实机接线、设备在线情况的核对。

## 对照源码

| 关系 | 主要依据 |
| --- | --- |
| 默认构建与实验构建互斥关系 | `CMakeLists.txt`、`Core/Src/main.c` |
| 任务创建 | `Core/Src/freertos.c`、`User/app/Src/app_control.c`、`app_debug.c`、`app_tof.c` |
| 正式控制顺序 | `User/app/Src/app_control.c`、`User/task/Src/task_*.c` |
| 视觉接收与确认 | `User/task/Src/task_vision.c`、`User/drv/Src/drv_hc13.c` |
| HX8 执行与反馈 | `User/task/Src/task_motion.c`、`User/drv/Src/drv_fsus.c` |
| 测距采样与快照 | `User/app/Src/app_tof.c`、`User/task/Src/task_tof_gate.c` |
| DataHub 状态模型 | `User/global/Inc/datahub.h`、`User/global/src/datahub.c` |
| 引脚与链路边界 | [pinmap.md](../hardware/pinmap.md)、[CURRENT_SYSTEM.md](CURRENT_SYSTEM.md) |

重新生成：在项目根目录执行 `python tools/draw_current_architecture.py`。脚本依赖 Pillow 及 Windows 微软雅黑字体，同时输出 PNG 与 SVG；SVG 文字保留可编辑性。
