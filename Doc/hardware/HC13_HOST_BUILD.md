# HC13统一调试固件 V2

2026-10-30：代码、ARM编译和新版desktop_app软件互通完成。**按用户要求未烧录，未驱动实物。**

## 对应项目与接口

- MCU：AerialRoboArm，`HC13_HOST_LINK=ON` 构建。
- PC：`C:\Users\Vincent Lu\Downloads\desktop_app`，实际V2，模块仍叫v1_protocol/v1_state。
- USART1 PB6/PB7：JY61P，启动配置115200、约200 Hz。
- USART2 PA2/PA3：HX8/UC-01，115200。
- USART3 PB10/PB11：HC13，230400、8N1，统一所有PC控制和无线回传。
- PA0 TIM2 CH1：SERVO1，333 Hz，沿用PTK分段脉宽。PA1/SERVO2不启动。
- 不依赖ELRS。RTT保留诊断和完整记录，但不执行此构建中的旧RTT角度命令。

## 编译，不烧录

```powershell
.\tools\build_hc13_host.ps1
```

输出：`cmake-build-hc13-host/FOC_DEMO2.elf`、`.hex`、`.bin`。

该脚本不调用OpenOCD，不烧录。选项为HX8_JY60_CAPTURE=ON、JY61P_CAPTURE=ON、HC13_HOST_LINK=ON、JY61P_DIAGNOSTIC=OFF。HC13_HOST_LINK默认OFF，旧采集/完整控制程序仍能分别编译。

当前Debug镜像约FLASH35.4 KB、RAM13.8 KB，处于64 KB/20 KB范围内。host构建RTT缓冲4 KB，旧独立采集构建仍8 KB。

## V2流程

完整线格式和状态语义见 [HC13_HOST_PROTOCOL_V2.md](./HC13_HOST_PROTOCOL_V2.md)。

| 操作 | 新行为 |
| --- | --- |
| MODE | 只切换路由，清除旧候选/准备；不移动、不打开夹爪 |
| HOME/NORMAL/PREP | 同一个竖直位0°→新鲜反馈确认→开爪PWM→等待标定时间→READY、DONE |
| PICK | 一次锁定目标/速度/3～5°提前量的联动动作，必须先READY |
| V | mdeg，只保存500 ms内候选，不直接运动 |
| TRIGGER | 触发一次候选PICK，消费候选 |
| MOVE | 普通位置调试，不自动闭爪 |
| 折叠 | 先闭爪、等待、再移动到已标定后折位置 |
| STOP/故障/模式退出 | 中止待闭爪、准备和候选，保持HX8；不回放旧目标、不自动松爪 |

闭爪只由新的实际位置反馈越过阈值触发，一次最多一次。PICK期间请求5 ms反馈轮询，受UART帧间隔和硬件刷新限制；不声称实际达到200 Hz。更快轮询不缩短到位约200 ms安静确认。

上电没有自动归零或PWM输出。传感器启动期间的缓存控制被丢弃，等待PC重新查询，不执行旧请求。

## 当前配置和必要标定

配置集中在 `User/app/Inc/app_hc13_host_config.h`：

- 竖直位0°，HOME与NORMAL同源，负向前伸，普通位置软件限位−90°～+85°。
- 最高500°/s软件上限、MOVE/PICK加减速各100 ms；准备和折叠30°/s、加减速各500 ms。
- SERVO1张开135°、闭合90°，只有命令，没有夹持/实际角度反馈。
- 默认提前闭爪4°，PICK允许3～5°。
- **用户确认没有实测开爪、闭爪等待时间，故HOST_OPEN_WAIT_MS/HOST_CLOSE_WAIT_MS保持−1，对外NA。**
- 后折位置仍未标定，HOST_FOLD_CALIBRATED=0，对外NA。

因此默认程序可以握手、显示传感器、使用独立调试指令；准备、联动抓取和折叠会按所需标定返回NOT_CALIBRATED。不能把测试用的100/150 ms或0 ms写入正式配置冒充实测值。135°预设也需在实际机构上确认方向。

## 传感器与记录

- MANUAL、VISION_DEMO约20 Hz ACC；VISION_PICK停传无线传感器流，内部IMU继续采样。
- STATE由GET STATE查询，PC约每秒查询；阶段变化使用MOTION，不高频刷状态。
- 普通MOVE/PICK可沿用RTT的完整200 Hz实验窗口；准备/折叠低速宏动作不混作普通高速出臂试次。
- 无线20 Hz显示不等于200 Hz存盘，RECORD ON和DONE也不代表文件已经保存。

## 测试

```powershell
.\tools\test_hc13_host.ps1
$env:HC13_NATIVE_PEER_EXE='C:\Users\Vincent Lu\Desktop\gitcode\AerialRoboArm\outputs\hc13_host_tests\test_hc13_host.exe'
Set-Location 'C:\Users\Vincent Lu\Downloads\desktop_app'
py -3.11 -m unittest discover -s tests -v
```

原生测试直接编译实际C模块，覆盖准备顺序、标定拒绝、采样跨阈值、一次闭爪、普通MOVE不闭爪、候选/触发、STOP及MODE取消、故障、发送背压和请求去重。

`tests/test_current_mcu_v2.py` 把真实上位机控制器通过模拟串口连接到实际C模块，不另造一份Python控制逻辑。测试端使用明确的模拟等待时间；另有未标定场景验证正式默认的拦截。全部HAL/执行器均模拟，不能代替实机联调。

上位机还修正了NORMAL流程收到READY后又被DONE错误清除准备状态的问题，并在独立夹爪/移动命令后同步取消旧READY。

## 待实机工作

本次没有烧录。后续先核对HC13电平、模块参数和INFO/CAL/STATE/ACC；再测开闭爪等待时间、确认135°/90°对应关系及后折位置；最后由用户明确启动机构动作。真实图传已有PC采集入口，泡沫识别算法仍需单独接入，不能把有图像当作已有抓取目标。
