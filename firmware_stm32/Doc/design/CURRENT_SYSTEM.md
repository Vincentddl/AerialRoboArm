# 当前系统：正式控制与无线实验

更新：2026-11-06。依据本仓库源码与默认编译选项描述，不将设计计划等同于实现或实测。

## 1. 职责与链路

| 部分 | 实际职责 |
| --- | --- |
| 遥控器/ELRS接收器 | 发送操作通道，CRSF串口接STM32 |
| 相机、模拟图传、电脑 | 获取图像、识别目标、根据标定映射目标角度 |
| HC13 | 传输视觉目标或V2调试消息，不传视频图像 |
| STM32 | 解析输入、模式与安全仲裁、行为/目标生成、执行与反馈监督 |
| HX8/UC-01 | 总线主臂执行器；HX8内部闭环并回报角度、电压等 |
| SERVO1 | PWM夹爪；无实际角度或夹持反馈 |
| VL53L1X | 正式构建中的独立测距与快照，不推断为已启用全部抓取闭环 |

主臂软件目标−90°～+85°。SERVO2/PA1默认禁用。当前不使用独立BLDC功率级、外接AS5600或MCU FOC环。

## 2. 正式程序运行

默认构建由`main.c`初始化外设和依赖，启动FreeRTOS。`freertos.c`创建应用任务。

| 任务 | 周期/间隔 | 作用 |
| --- | --- | --- |
| ControlTask | 20ms | 遥控读取、启动状态机、正常控制/故障处理；发布DataHub快照与心跳 |
| DebugTask | 100ms | RTT输入请求、状态打印与指示IO |
| TofTask | 2ms数据就绪轮询 | 有新数据时读测距，发布独立快照；不代表500Hz测距 |
| defaultTask | 1000ms | 维护、控制心跳检查与条件喂狗 |

```text
启动探测/配置/使能 → RUNNING
RUNNING每轮：遥控输入 → 视觉目标 → 链路安全检查 → 仲裁
            → 机械臂/夹爪状态机 → HX8运动与反馈 → DataHub/心跳
异常：进入对应保护/故障状态；按条件恢复，不自动等同于正常动作完成
```

### MANUAL

| 通道 | 语义 | 当前判定 |
| --- | --- | --- |
| CH1 | 主臂目标增量 | ±10%死区外按幅度累积；回中保持；满杆目标变化率150°/s |
| CH4 | 夹爪 | 正向>55%指令135°；回落<25%恢复90° |
| CH5/SA | 模式 | 高于中值MANUAL，否则AUTO |
| CH6/SE | 归零请求 | 低变高触发一次0°目标，受安全条件约束 |
| CH9/SB | 故障复位请求 | 进入高档产生约150ms脉冲；持续高档不重复触发 |
| CH10/SD | 急停 | 不高于中值时有效，高于中值时释放 |

SA/SB等是软件通道约定，物理摇杆/拨杆对应关系由遥控器混控决定。150°/s是目标变化率，不能直接写成舵机实测速度。

### AUTO

电脑持续发送目标。`TaskVision`解析后交仲裁器：目标存在、置信度≥60%、年龄<200ms才使用。有效时持续更新目标；无效时走保持分支。模式切换与输出还受执行器健康检查约束。

进入AUTO按正式程序预设130°开爪；随后CH4操作夹爪。`AUTO_GRIP_THREE_FRAME`默认OFF，可选ON时才根据新稳定帧闭爪。该参数不决定主臂是否等待三帧跟随。详见[自动模式](../hardware/AUTO_PICK_SEQUENCE.md)。

### 安全与反馈

急停、遥控失联、堵转、执行器离线与故障锁定优先。锁定故障需要原因消除、急停释放和复位请求等条件。正式路径的停止可能发送解除力矩指令，不等同于V2的停止保持；不能跨构建照搬安全行为。

HX8通过ServoMonitor提供编码器角度与电气状态；外部监督包含反馈有效性及有限位置微调。正式配置：手动速度参数220°/s、AUTO速度上限500°/s、加减速各100ms、反馈查询间隔100ms、目标发送间隔限制40ms。这些是配置，不是实测响应性能。任务20ms也不是1kHz运动闭环。

## 3. 无线V2构建

入口为`App_Hx8Jy60Capture_Run()`，执行采集主循环，不创建正式ControlTask。USART1接JY61P，控制经USART3/HC13。构建要求`HX8_JY60_CAPTURE=ON`、`JY61P_CAPTURE=ON`、`HC13_HOST_LINK=ON`和`JY61P_DIAGNOSTIC=OFF`。

| V2操作 | 行为 |
| --- | --- |
| GET/PING | 查询与心跳 |
| MOVE | MANUAL下独立角度动作，不自动闭爪 |
| GRIP/SERVO | 独立夹爪PWM操作 |
| STOP | 停止保持，根据设备反馈检查决定故障状态；上位机需查询确认 |
| PREP/HOME/NORMAL | 位置准备、开爪及等待，依赖标定 |
| PICK/TRIGGER | 单次联动抓取；V帧只更新候选 |

V2角度为毫度。正式视觉V帧用度并持续跟随，不能直接复用。V2标定中开爪135°、闭爪90°为预设，等待时间和后折位置为NA；未知项目不自动填入经验值。

IMU约200Hz目标采样，经RTT保存完整数据；无线ACC约20Hz。文件完整性以实际记录结果、序号和丢包检查为准。小舵机无位置反馈，APPLIED仅表示PWM命令已应用。

## 4. 证据与源码定位

协议/状态机代码和构建检查可用于确认实现。历史V2烧录及双向只读查询已经记录，但不能据此断言全部动作或抓取成功。PC视觉算法和上位机源码不在本仓库，标定的适用性需随外部项目核实。

- `User/app/Src/app_control.c`：正式控制入口和阶段管理。
- `User/task/Src/task_rc.c`、`task_vision.c`、`task_arbiter.c`、`task_manipulator.c`、`task_motion.c`：正式控制链。
- `User/mod/Src/mod_rc_semantic.c`：输入规则；`User/global/Inc/ara_prio.h`：周期。
- `User/mod/Src/mod_hc13_host.c`及`User/app/Src/app_hx8_jy60_capture.c`：V2/实验链。

保留DAPLink、OpenOCD、SEGGER RTT、现用HAL/CMSIS/FreeRTOS。工程名FOC_DEMO2仅为工具兼容。构建与接线见[README](../../README-zh.md)和[引脚表](../hardware/pinmap.md)。

## 5. 当前架构图（2026-11-14）

- [五分区分层架构 PNG（保留原图布局）](../../Figures/architecture/control_system_five_columns_20261114.png) / [可编辑 SVG](../../Figures/architecture/control_system_five_columns_20261114.svg)
- [控制系统总体架构 PNG](../../Figures/architecture/control_system_current_20261114.png) / [可编辑 SVG](../../Figures/architecture/control_system_current_20261114.svg)
- [软件分层架构 PNG](../../Figures/architecture/software_architecture_current_20261114.png) / [可编辑 SVG](../../Figures/architecture/software_architecture_current_20261114.svg)
- [架构图依据与边界](architecture_figures_20261114.md)

上述图对应默认正式控制构建；旧图仅保留为历史资料，不作为当前系统实现说明。
