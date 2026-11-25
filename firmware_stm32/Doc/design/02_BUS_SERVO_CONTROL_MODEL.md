# 总线舵机控制理论建模说明

本文档说明在不涉及图像识别、图传、空间坐标估计的前提下，如何对本项目中的总线舵机控制链路进行控制理论建模。本文关注对象是 STM32 通过 USART2、UC-01 转接板控制 HX8-U26H-M / FSUS 兼容总线舵机的主臂控制路径。

## 1. 建模对象与边界

本项目中的主臂执行器是带内部控制器的总线舵机。STM32 并不直接控制电机电压、电机相电流或功率级 PWM，而是通过串口总线发送运动命令。

因此，建模对象不应定义为：

```text
电压 -> 电流 -> 电磁转矩 -> 转子角度
```

而应定义为：

```text
目标角度 / 速度 / 加减速参数
  -> 总线通信
  -> 舵机内部位置闭环
  -> 输出轴角度 / 电流 / 功率 / 状态反馈
```

在控制理论上，可以将总线舵机视为一个已经封装好的内部闭环位置执行器。外部控制层只需要建立“命令输入到角度输出”的执行器级模型。

## 2. 项目中的控制链路

当前主臂总线舵机控制链路为：

```text
上层模式/业务决策
  -> TaskArbiter
  -> TaskManipulator
  -> MotionCmd
  -> TaskMotion
  -> DrvFsus_EncodeSetAngleByVelocity()
  -> BSP_UART_FSUS transaction
  -> STM32 USART2
  -> UC-01 转接板
  -> HX8 / FSUS 总线舵机
  -> ServoMonitor / QueryAngle feedback
```

对应代码入口：

| 层级 | 文件 | 作用 |
| --- | --- | --- |
| 协议层 | `User/drv/Inc/drv_fsus.h` | FSUS 帧编码、解码、协议常量。 |
| 运动代理层 | `User/task/Inc/task_motion.h` | 将上层 `MotionCmd_t` 转换成总线舵机事务。 |
| 串口事务层 | `User/bsp/Inc/bsp_uart.h` | USART2 总线发送、接收、等待回包。 |
| 业务层 | `User/task/Src/task_manipulator.c` | 根据 Arbiter 输出生成主臂目标角。 |
| 状态观测 | `User/app/Src/app_debug.c` | 输出 `pos/tgt/wr/rd/load` 等调试快照。 |

## 3. 输入、输出与状态变量

### 3.1 输入变量

总线舵机外部控制输入可以定义为：

```text
u(t) = [theta_cmd, v_cmd, t_acc, t_dec, power]
```

其中：

| 变量 | 含义 | 当前项目来源 |
| --- | --- | --- |
| `theta_cmd` | 目标角度，单位 deg | `MotionCmd_t.target_angle_deg` |
| `v_cmd` | 最大运动速度，单位 deg/s | `MotionCmd_t.velocity_deg_per_s` |
| `t_acc` | 加速时间，单位 ms | `MotionCmd_t.t_acc_ms` |
| `t_dec` | 减速时间，单位 ms | `MotionCmd_t.t_dec_ms` |
| `power` | 执行功率限制，单位 mW，`0` 表示舵机默认 | `MotionCmd_t.power_mw` |

### 3.2 输出变量

反馈输出可以定义为：

```text
y(t) = [theta, I, P, status]
```

其中：

| 变量 | 含义 | 当前项目来源 |
| --- | --- | --- |
| `theta` | 当前舵机角度，单位 deg | `FsusFeedback_t.angle_deg` |
| `I` | 当前电流，单位 mA | `FsusFeedback_t.current_ma` |
| `P` | 当前功率，单位 mW | `FsusFeedback_t.power_mw` |
| `status` | 舵机状态位 | `FsusFeedback_t.status` |
| `voltage` | 供电电压，单位 mV | `FsusFeedback_t.voltage_mv` |

### 3.3 主要控制误差

最核心的误差是角度跟踪误差：

```text
e(t) = theta_cmd(t) - theta(t)
```

工程上还需要关注：

```text
e_ss  = 稳态误差
L     = 命令到响应的延迟
T_s   = 调节时间
M_p   = 超调量
I_max = 运动过程峰值电流
```

## 4. 舵机级模型结构

由于总线舵机内部已经包含电机驱动、编码器、位置控制器和保护逻辑，外部控制理论建模推荐采用黑箱模型或灰箱模型。

### 4.1 一阶惯性模型

最基本的模型是：

```text
G(s) = theta(s) / theta_cmd(s) = K / (tau*s + 1)
```

其中：

| 参数 | 含义 |
| --- | --- |
| `K` | 静态增益。理想情况下接近 `1`。 |
| `tau` | 时间常数。越小表示响应越快。 |

该模型适合描述舵机在不过载、不饱和、目标角度变化较小的情况下的跟踪行为。

### 4.2 带延迟的一阶模型

总线舵机存在通信延迟、命令解析延迟和内部响应延迟，因此更推荐使用：

```text
G(s) = K * e^(-L*s) / (tau*s + 1)
```

其中：

| 参数 | 含义 |
| --- | --- |
| `L` | 总延迟，包括串口发送、舵机解析、内部启动响应。 |
| `tau` | 响应时间常数。 |
| `K` | 静态增益。 |

在实验中可以通过“命令发出时刻”和“反馈角度开始明显变化时刻”估计 `L`。

### 4.3 离散时间模型

如果控制算法在 MCU 周期任务中实现，更方便使用离散模型：

```text
theta[k+1] = theta[k] + alpha * (theta_cmd[k-d] - theta[k])
```

其中：

| 参数 | 含义 |
| --- | --- |
| `alpha` | 离散响应系数，`0 < alpha < 1`。 |
| `d` | 延迟拍数。 |
| `k` | 离散采样序号。 |

如果采样周期为 `T_samp`，一阶模型中的 `tau` 与 `alpha` 近似关系为：

```text
alpha = 1 - exp(-T_samp / tau)
```

## 5. 限速与加减速建模

总线舵机命令不是单纯的目标位置命令，还包含速度和加减速约束。因此模型必须包含饱和与轨迹规划。

### 5.1 角度限幅

根据当前 FSUS 协议层约束：

```text
theta_cmd ∈ [-180 deg, +180 deg]
```

外部控制层应先做：

```text
theta_ref = clamp(theta_cmd, -180, +180)
```

### 5.2 速度限幅

协议层定义速度范围：

```text
v_cmd ∈ [1, 750] deg/s
```

舵机角速度满足：

```text
|d(theta)/dt| <= v_cmd
```

离散形式：

```text
delta_theta_max = v_cmd * T_samp
theta[k+1] = theta[k] + clamp(theta_ref[k] - theta[k],
                              -delta_theta_max,
                              +delta_theta_max)
```

示例：

```text
v_cmd = 300 deg/s
T_samp = 0.02 s
delta_theta_max = 6 deg
```

这意味着理论上一拍最多变化约 `6 deg`。

### 5.3 加减速约束

协议层要求加减速时间不小于：

```text
t_acc >= 20 ms
t_dec >= 20 ms
```

近似加速度：

```text
a_max = v_cmd / t_acc
d_max = v_cmd / t_dec
```

如果：

```text
v_cmd = 300 deg/s
t_acc = 100 ms = 0.1 s
```

则：

```text
a_max = 3000 deg/s^2
```

因此总线舵机的运动可以理解为：

```text
目标角度
  -> 梯形/三角速度轨迹规划
  -> 内部位置闭环跟踪
  -> 输出轴角度
```

## 6. 通信延迟建模

总线控制不同于直接 PWM 控制。每次命令需要经过串口帧传输和回包等待。

总延迟可以分解为：

```text
L = L_encode + L_tx + L_parse + L_servo_start + L_feedback
```

工程上通常不逐项精确建模，而是通过实验直接测量总延迟：

```text
L = t_motion_start - t_command_sent
```

其中：

| 时间点 | 定义 |
| --- | --- |
| `t_command_sent` | STM32 发出 SetAngleByVelocity 命令的时间。 |
| `t_motion_start` | 反馈角度开始变化，或电流明显上升的时间。 |

当前项目的 ServoMonitor 反馈周期为：

```text
TASK_MOTION_FEEDBACK_PERIOD_MS = 100 ms
```

因此如果只依赖 ServoMonitor 观察角度，测得的延迟分辨率较粗。更精确的实验需要提高反馈采样率，或使用外部编码器/逻辑分析仪辅助。

## 7. 阶跃响应实验

建模最常用的方法是阶跃响应辨识。

### 7.1 实验输入

从一个稳定角度跳到另一个稳定角度：

```text
theta_cmd: 0 deg -> 30 deg
```

建议测试多个幅值：

```text
0 -> 10 deg
0 -> 30 deg
0 -> 60 deg
0 -> 90 deg
```

也建议测试多个速度：

```text
v_cmd = 100, 300, 500, 750 deg/s
```

以及多个加速度时间：

```text
t_acc = 20, 80, 200 ms
t_dec = 20, 80, 200 ms
```

### 7.2 记录数据

建议记录如下表格：

| `t_ms` | `theta_cmd` | `theta_actual` | `error` | `current_ma` | `power_mw` | `status` | `wr` | `rd` |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 0 | 0.1 | -0.1 | 80 | 500 | 0 | 0 | 0 |
| 100 | 30 | 5.8 | 24.2 | 320 | 2800 | 0 | 0 | 0 |
| 200 | 30 | 18.4 | 11.6 | 410 | 3600 | 0 | 0 | 0 |
| 300 | 30 | 28.9 | 1.1 | 180 | 1300 | 0 | 0 | 0 |

其中：

```text
wr = last_write_result
rd = last_read_result
```

`wr/rd` 用于区分控制效果问题和通信问题。

### 7.3 提取动态指标

#### 延迟

```text
L = t_motion_start - t_command_sent
```

#### 时间常数

一阶系统达到最终变化量的 `63.2%` 时，对应时间近似为 `tau`。

如果：

```text
theta_start = 0 deg
theta_final = 30 deg
```

则 `63.2%` 响应点为：

```text
theta_63 = 0 + 0.632 * 30 = 18.96 deg
```

若舵机在 `120 ms` 达到 `18.96 deg`，且延迟 `L = 30 ms`，则：

```text
tau ≈ 120 ms - 30 ms = 90 ms
```

#### 稳态误差

```text
e_ss = theta_cmd - theta_final
```

例如：

```text
theta_cmd = 30 deg
theta_final = 29.4 deg
e_ss = 0.6 deg
```

#### 超调量

```text
M_p = (theta_peak - theta_cmd) / |theta_cmd - theta_start| * 100%
```

例如：

```text
theta_peak = 32 deg
theta_cmd = 30 deg
theta_start = 0 deg
M_p = 6.7%
```

#### 调节时间

调节时间可定义为误差进入并保持在某个容差范围内的时间：

```text
|theta_cmd - theta_actual| <= epsilon
```

例如：

```text
epsilon = 1 deg
```

## 8. 抖动原因的控制理论解释

总线舵机抖动通常不是单一原因，而是由外部命令、通信、内部闭环和机械结构共同造成。

常见原因包括：

| 原因 | 控制理论解释 | 现象 |
| --- | --- | --- |
| 高频重复发送目标 | 外部命令持续打断舵机内部轨迹规划 | 舵机持续微动、发热 |
| 目标角小幅来回变化 | 目标信号含高频噪声 | 在目标附近抖动 |
| 无死区 | 微小误差也触发新动作 | 到位后仍频繁修正 |
| 通信延迟 | 外部控制基于滞后反馈做判断 | 命令与实际状态错位 |
| 机械间隙 | 位置反馈变化不连续 | 换向时抖动或迟滞 |
| 负载扰动 | 重力矩/摩擦导致内部闭环补偿 | 某些角度更容易抖 |
| 供电不足 | 电压下跌导致力矩不足 | 回包超时、角度不稳 |

## 9. 外部控制策略

由于总线舵机内部已经有位置闭环，STM32 外部不建议再做高频 PID 直接追踪误差。更推荐做目标生成和命令整形。

### 9.1 命令去重

如果目标角没有明显变化，不重复发送命令：

```text
if abs(theta_new - theta_last_sent) < epsilon_cmd:
    do not send
```

推荐初始值：

```text
epsilon_cmd = 1 deg
```

### 9.2 死区

如果当前误差已经足够小，认为到位：

```text
if abs(theta_cmd - theta_actual) < epsilon_arrive:
    arrived = true
```

推荐初始值：

```text
epsilon_arrive = 1..2 deg
```

### 9.3 目标限速

限制外部目标角变化速度：

```text
theta_cmd_limited[k] =
    theta_cmd_limited[k-1]
    + clamp(theta_cmd_raw[k] - theta_cmd_limited[k-1],
            -max_step_deg,
            +max_step_deg)
```

推荐初始值：

```text
max_step_deg = 3..10 deg per control update
```

### 9.4 低通滤波

对上层目标做一阶低通：

```text
theta_f[k] = beta * theta_f[k-1] + (1 - beta) * theta_raw[k]
```

其中：

```text
0 < beta < 1
```

`beta` 越大，目标越平滑，但响应越慢。

### 9.5 故障保护

根据反馈状态进行保护：

```text
if status indicates stall:
    reduce speed or stop

if current_ma too high:
    reduce power or stop

if rd == TIMEOUT repeatedly:
    mark servo offline
```

当前项目已经有在线宽限和回读错误码：

```text
wr = last_write_result
rd = last_read_result
```

其中 `rd=5` 表示回包超时。

## 10. 推荐的项目级模型

综合当前代码和总线舵机特性，推荐使用如下项目级模型：

```text
theta_ref[k] = clamp(theta_cmd[k], -180, +180)

if abs(theta_ref[k] - theta_last_sent) < epsilon_cmd:
    no new FSUS command
else:
    send SetAngleByVelocity(theta_ref,
                            v_cmd,
                            t_acc,
                            t_dec,
                            power)

servo response:
    G(s) = K * e^(-L*s) / (tau*s + 1)

constraints:
    |d(theta)/dt| <= v_cmd
    |d2(theta)/dt2| <= v_cmd / t_acc
```

这套模型的含义是：

1. 上层只生成目标角，不直接控制舵机内部电机。
2. 外部控制层负责限幅、限速、死区、命令去重和故障保护。
3. 舵机内部负责执行位置闭环。
4. 通过实验辨识 `K`、`L`、`tau`、`e_ss`、`T_s` 等参数。

## 11. 专业建模流程

完整流程如下：

```text
1. 明确对象
   总线舵机作为内部闭环位置执行器。

2. 定义输入输出
   输入：theta_cmd, v_cmd, t_acc, t_dec, power
   输出：theta, current, power, status

3. 选择模型结构
   带延迟一阶模型 + 限速 + 加减速约束。

4. 设计实验
   做不同幅值、不同速度、不同负载下的阶跃响应。

5. 采集数据
   记录时间、目标角、反馈角、电流、功率、状态、通信错误码。

6. 参数辨识
   提取 L, tau, K, e_ss, M_p, T_s。

7. 建立外部控制策略
   死区、命令去重、低通滤波、目标限速、安全保护。

8. 验证模型
   比较模型预测曲线和真实反馈曲线。

9. 调整策略
   根据抖动、超调、响应慢、过流等现象修改速度、加速度、死区和更新频率。
```

## 12. 报告或论文中的表述建议

可以写成：

> 本系统所使用的 HX8 总线舵机为内部闭环位置伺服执行器，主控 STM32 不直接控制电机电压或电流，而是通过 FSUS 串口协议下发目标角度、速度、加减速时间和功率限制。因厂家内部控制器参数不可见，本文将该总线舵机抽象为带通信延迟、速度饱和和加减速约束的一阶位置执行器。系统通过 ServoMonitor 回读角度、电流、功率和状态位，并在外部控制层实现角度限幅、命令去重、死区、目标限速和异常保护，以避免重复命令和反馈延迟导致的舵机抖动。

数学模型可表述为：

```text
G(s) = K * e^(-L*s) / (tau*s + 1)
```

并满足：

```text
theta_cmd ∈ [-180 deg, 180 deg]
|d(theta)/dt| <= v_cmd
|d2(theta)/dt2| <= v_cmd / t_acc
```

其中 `K`、`L`、`tau` 由阶跃响应实验辨识获得。

## 13. 与 PWM 舵机的区别

本项目还包含 PTK 7462W PWM 舵机。PWM 舵机同样是内部闭环位置执行器，但输入不是总线协议帧，而是 PWM 脉宽：

```text
0 deg   -> 500 us
90 deg  -> 1520 us
180 deg -> 2500 us
```

其静态映射为：

```text
0 <= theta <= 90:
    pulse = 500 + theta * 1020 / 90

90 < theta <= 180:
    pulse = 1520 + (theta - 90) * 980 / 90
```

PWM 舵机通常缺少角度、电流、温度等总线反馈，因此更适合做简单末端执行器控制；HX8/FSUS 总线舵机更适合做可观测、可保护、可建模的主臂关节控制。

## 14. 建模结论

不涉及图像时，本项目总线舵机控制理论建模的核心结论是：

```text
总线舵机 = 带通信延迟 + 限速 + 加减速约束 + 内部闭环的位置执行器
```

外部 STM32 控制层不应高频重复执行位置 PID，而应负责：

```text
目标角生成
角度限幅
速度/加速度约束
命令去重
死区抑制
状态监测
故障保护
```

通过阶跃响应实验得到模型参数后，就可以解释和优化舵机响应速度、稳态误差、超调、抖动和过载问题。
