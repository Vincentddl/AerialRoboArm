# 项目文档导航

请先选择当前使用的固件链路。本文档只把现行实现列为主入口，历史规划不再作为使用说明。

## 正式控制：ELRS手动＋电脑视觉目标

1. [系统架构、任务与模式](design/CURRENT_SYSTEM.md)：职责边界、MANUAL/AUTO、安全与反馈。
2. [实际接线](hardware/pinmap.md)：三路UART、PWM、测距及SWD。
3. [自动模式与视觉目标格式](hardware/AUTO_PICK_SEQUENCE.md)：持续跟随、CH4夹爪、可选三帧配置。
4. [启动角度保护](hardware/HX8_STARTUP_ANGLE_GUARD.md)、[位置微调](hardware/HX8_POSITION_TRIM.md)：正式控制的执行监督。
5. [控制台调试](hardware/VOFA_MANUAL_WITHOUT_ELRS.md)、[RTT日志](hardware/RTT_LOG_MODES.md)：正式程序调试入口。

## 无线调试：HC13 V2＋HX8＋JY61P

1. [V2构建与对接](hardware/HC13_HOST_BUILD.md)：编译、接入、查询、动作和恢复边界。
2. [V2完整协议](hardware/HC13_HOST_PROTOCOL_V2.md)：ASCII帧、单位、回执与状态字段。
3. [JY61P配置与采集](hardware/JY61P_CAPTURE.md)：模块设置、速率检查、原始记录。文内旧RTT运动命令只适用于独立采集构建。
4. [同步采集格式](hardware/HX8_JY60_CAPTURE.md)：记录文件和事件语义。

## 硬件和专项工具

- [HC13配置](hardware/hc13_config.md)：两端参数及半双工约束。
- [DAPLink/RTT排障](hardware/rtt_boot_troubleshooting.md)：日志访问与启动检查。
- [SERVO1独立调试](hardware/SERVO1_ONLY_TEST.md)、[JY61P诊断](hardware/JY61P_DIAGNOSTIC.md)、[JY60资料](hardware/JY60_IMU.md)、[JY60独立读取](hardware/JY60_UART_DEMO.md)：仅在选择相应构建时使用。
- [VL53L1X](hardware/vl53l1x_grasp_gate.md)：正式构建的测距快照任务，不能据此声称测距已参与所有抓取准入。
- [HX8模型研究](design/02_BUS_SERVO_CONTROL_MODEL.md)：建模参考，不是现行固件参数规范。
- [通信时延测法](hardware/HC13_LATENCY_TEST.md)：专用诊断试验，不等于整机控制时延。

## 数据与维护

相机识别、角度标定和上位机是外部电脑项目。原始数据在本机`outputs/`，不随Git代码上传；`Figures/DEMO`等历史演示不直接作为当前构建实测证据。

2026-11-06清理记录见[历史文件清理](MAINTENANCE_CLEANUP_20261106.md)。已移除过时BLDC/AS5600文档、旧V1协议、重复方案和设计草案；更早版本可从Git历史恢复，未重写提交历史。
