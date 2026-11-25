# HC13 V2无线调试构建与对接

用途：电脑经两块HC13控制HX8和夹爪，回传状态及JY61P加速度；独立于正式ELRS/视觉控制构建。更新：2026-11-06。

## 编译与入口

```powershell
.\tools\build_hc13_host.ps1
.\tools\test_hc13_host.ps1
```

要求CMake、Ninja、Arm GNU工具链；原生测试要求GCC（脚本查PATH或已知CLion路径）。脚本配置`HX8_JY60_CAPTURE=ON`、`JY61P_CAPTURE=ON`、`HC13_HOST_LINK=ON`、`JY61P_DIAGNOSTIC=OFF`，产物为`cmake-build-hc13-host/FOC_DEMO2.elf/.hex/.bin`，只编译不烧录。

`main.c`进入`App_Hx8Jy60Capture_Run()`主循环。USART1接JY61P、USART2接HX8、USART3接HC13；详见[接线表](pinmap.md)。上电不自动归零或启动夹爪PWM，显式指令才应用PWM。机构控制使用HC13 V2，旧RTT `g/p`命令不是此构建的控制入口。

## 只读通信检查

电脑选择实际HC13串口，230400、8N1、无流控；打开前关闭DTR/RTS。同一端口只能由一个程序占用。先读取INFO/CAL/STATE，再确认PING/PONG，不能仅凭收到ACC就认为所有控制路径成功。

```text
GET,101,INFO
GET,102,CAL
GET,103,STATE
PING,104
```

发送示例以LF结束。读到匹配请求编号的回复才算本次查询往返成功；COM号不固定。裸`ERROR`不是V2的`ERR,id,code`，应检查选错端口、模块模式或串口控制线状态。

## 手动控制与超时恢复

```text
MOVE,105,-30000,100
GRIP,106,OPEN
GRIP,107,CLOSE
STOP,108
GET,109,STATE
```

以上只是格式示例，不是自动测试序列。MOVE是−30°、100°/s，需MANUAL和设备就绪；GRIP使用MCU配置角度；STOP尝试停止并保持。动作需按实际机构条件由操作者明确发起。上位机“低速归零”目前发送`MOVE,id,0,30`，是普通限位内运动，不是独立采集固件的越界恢复`h`。

MOVE接收ACK后仍需等待DONE/故障及实际角度。运动超时会留下故障状态，上位机不能只清除报错；恢复复用STOP，由MCU检查反馈，并在ACK后重新GET STATE确认。不会自动重发旧动作或归零，底层问题未消失时仍可能失败。PWM的APPLIED仅说明已设置命令。

## 联动流程与未知标定

配置位于`User/app/Inc/app_hc13_host_config.h`：竖直位0°，张开135°、闭合90°，提前量4°；开闭爪等待时间−1、后折位置未标定，对外NA。

HOME/PREP/NORMAL、PICK/TRIGGER和折叠依赖各自标定，不把测试模拟值写成实测参数。MODE只切路由，不自动开爪；V以毫度存候选，不直接驱动主臂。正式控制的度单位持续跟随V不能用于此处。完整格式见[协议V2](HC13_HOST_PROTOCOL_V2.md)。

## 采集和验证边界

内部IMU目标200Hz，MANUAL/VISION_DEMO无线ACC约20Hz，VISION_PICK停发无线ACC但内部采样继续。RTT1经9091读取完整采样；需运行`tools/capture_hx8_jy60.py`等采集工具并检查生成文件。RECORD ON、ACC或DONE都不保证电脑记录完整。

现有软件测试和构建已通过，历史烧录校验、双向查询和ACC接收有记录；未据此认定全部机械动作/抓取成功。外部电脑上位机和识别模型不在此MCU仓库，曾分发的修改版为`desktop_app_wireless_20261101`，对接时仍须核对实际启动目录及协议版本。
