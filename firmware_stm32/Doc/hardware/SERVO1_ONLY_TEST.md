# SERVO1 单独测试

使用独立构建目录 `cmake-build-servo1-only`，CMake 参数 `SERVO1_ONLY_TEST=ON`。只执行 PA0 / TIM2_CH1 的夹爪 PWM；不调用 TaskMotion_Update，不执行 HX8 启动归零、遥控主臂、视觉跟随或 SERVO2/PA1。

上电默认 90°，不自动扫角、不需要 ELRS 或 HX8 回包。通过 RTT 测试：

```text
p 0 130
p 0 90
```

建议只在确认接线、共地和供电后使用这两个现有开合角。测试模式 `P` 或 `p 0 -1` 恢复 90°默认。`g` 和数字主臂预设会提示 HX8 已禁用。

启动标记：`[FW] SERVO1_ONLY PA0 PWM; HX8 and SERVO2 disabled`。
运行日志：`[SERVO1_ONLY] angle=130 pin=PA0 servo2=off hx8=off`。

烧录入口：`tools/flash_servo1_only.bat`。恢复完整上电归零及视觉控制时使用 `tools/flash_and_rtt.bat`，它使用原 `cmake-build-debug`，不要将 SERVO1_ONLY_TEST 开在该正常目录。

小舵机电源与 STM32 主板电源都必须存在，并共地。DAPLink 被电脑识别不代表 MCU 有供电。独立固件不会给 HX8 发送停止指令，因此测试时可断开 HX8 电源；机械臂应承托稳定。只保持 MCU 和 SERVO1 供电时仍应确认两者电压符合各自要求。

2026-10-10：前两次连接检查失败于 `cannot read IDR`，随后恢复连接并完成烧录及校验。板上已输出 SERVO1_ONLY 标记；已发送 `p 0 130`，日志由 90°变为 130°。实物转动仍等待观察，软件输出不能代替舵机动作验证。
# 运行时按键调试（2026-10-12）

完整固件无需切换烧录版本。VOFA 文本发送，换行 `\n`：

| 命令 | 行为 |
|---|---|
| `s` | 切换 SERVO1 独立调试；进入时先请求 HX8 Stop/Unlock，SERVO1 设为 90° |
| `h` | 选择 HX8 调试入口；如正在调试小舵机，则退出并重新进行完整启动检查，然后可用 `g <角度>` |
| `4` | 独立调试中设为 90° |
| `5` | 独立调试中设为 130° |
| `p 120` | 独立调试中将 SERVO1 设为 120°；范围 0..180° |
| `p 0 120` | 兼容原来的通道格式 |
| 再发 `s` | 退出独立调试，重新检查 HX8/ELRS 并执行正常启动回零 |

独立调试循环只更新 SERVO1 PWM，不运行 HX8 运动、视觉跟随、回零或姿态舵机控制。不依赖 ELRS 和 HX8 正常反馈。HX8 停止指令在通信中断时无法保证到达；夹爪无角度反馈，`angle_cmd` 是指令。SERVO2 默认禁用。独立调试中 `g`/大舵机预设命令被拒绝。

保留原来的简洁/详细状态打印，并追加每秒输出 `[SERVO1] angle_cmd=90 pulse_us=1520 pwm_hz=333 hx8_control=OFF feedback=N/A`。重启恢复完整模式，不保存调试模式。退出后正常回零可能移动大舵机，须保持运动范围无遮挡。

模式和调试对象分开显示：小舵机入口是 `[DBG] MANUAL DEBUG ... debug=SERVO1`，HX8 入口显示 `debug=HX8`，默认 `debug=NONE`。模式名称不再使用 `SERVO1_DBG`。HX8 入口不自动发角度，也不绕过回零、供电、反馈和 ELRS 的启动条件；启动未通过时仍显示 INIT/ERROR，以免把未就绪误报为正常 MANUAL。

