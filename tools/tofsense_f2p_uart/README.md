# TOFSense-F2 P 独立 UART 测试程序

这是一个与 AerialRoboArm STM32 固件完全解耦的电脑端串口工具，用于读取和解析
TOFSense-F2 P 的主动输出或查询输出。它不会参与仓库根目录的 CMake 构建。

## 1. 主要参数

| 参数 | 规格 |
| --- | --- |
| 测距方式 | 单点 TOF 激光测距 |
| 测距范围 | 0.05-40 m |
| 距离分辨率 | 1 mm |
| 最高测量频率 | 100 Hz |
| 测距精度 | 标准差 3 mm；均方根误差 3 cm |
| 视场角 FOV | 1-2° |
| 激光波长 | 905 nm |
| 抗环境光 | 约 100 klux |
| 供电 | 4.3-5.2 V，约 250 mW |
| 信号电平 | 3.3 V TTL |
| 接口 | UART、IIC、I/O（共用物理接口） |
| UART 默认配置 | 921600 bps；程序使用 8-N-1 |
| 工作温度 | -10-60 °C |
| 存储温度 | -30-60 °C |
| 尺寸 | 22.7 x 28.0 x 13.6 mm |
| 重量 | 7.5 g |

注意：供电电压是 4.3-5.2 V，但 UART 信号是 3.3 V TTL，不要把 5 V 逻辑电平接到模块信号脚。

## 2. UART 接线

GH1.25 4P 母座卡扣凹槽朝上时，从右向左为：`VCC、GND、RX、TX`。

连接 USB-TTL 时应交叉连接数据线：

| TOFSense-F2 P | USB-TTL |
| --- | --- |
| VCC | 5 V（必须处于 4.3-5.2 V 范围） |
| GND | GND |
| RX | TX |
| TX | RX |

USB-TTL 必须稳定支持 921600 bps。官方建议 CP2102、CH343 或支持该高速率的 CH340。

## 3. 16 字节主动输出帧

数据采用小端序，校验和是前 15 个字节之和的低 8 位。

| 字节偏移 | 长度 | 含义 |
| --- | ---: | --- |
| 0 | 1 | 帧头 `0x57` |
| 1 | 1 | Frame0 标识 `0x00` |
| 2 | 1 | 保留字段，示例为 `0xFF` |
| 3 | 1 | 模块 ID |
| 4-7 | 4 | 模块上电时间，ms |
| 8-10 | 3 | 有符号距离原始值，除以 1000 得到 m |
| 11 | 1 | 距离状态：F2 系列 `0=无效，1=有效` |
| 12-13 | 2 | 信号强度，0-65535 |
| 14 | 1 | `range_precision` 原始值；当前公开资料未给出 F2 P 的换算定义 |
| 15 | 1 | 校验和 |

查询模式的命令为 `57 10 FF FF <ID> FF FF <CHECKSUM>`。例如 ID 为 3 时：
`57 10 FF FF 03 FF FF 66`。

## 4. 安装与运行

```powershell
cd tools/tofsense_f2p_uart
python -m pip install -r requirements.txt
python tofsense_f2p_uart.py --self-test
python tofsense_f2p_uart.py --list-ports
```

主动输出模式：

```powershell
python tofsense_f2p_uart.py --port COM6 --raw
```

读取 100 帧后退出：

```powershell
python tofsense_f2p_uart.py --port COM6 --count 100
```

查询输出模式（模块 ID 为 3，每 100 ms 查询一次）：

```powershell
python tofsense_f2p_uart.py --port COM6 --query-id 3 --query-period 0.1
```

Linux 下把 `COM6` 换成类似 `/dev/ttyUSB0` 的设备名。

## 5. 资料来源

- 本地数据手册：`TOFSense_F2_P_Datasheet_V1.0_zh.pdf`
- 官网协议：https://support.nooploop.com/cn/tofsense/protocol/
- 官网示例代码：https://support.nooploop.com/cn/tofsense/example-code/
- 官网硬件接线：https://support.nooploop.com/cn/tofsense/hardware/
- 官网 FAQ：https://support.nooploop.com/cn/tofsense/FAQ-for-tOfsense/
