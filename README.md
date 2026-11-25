# AerialRoboArm — 轻量化动平衡大行程机载折叠机械臂系统

本仓库汇总了**轻量化动平衡大行程快速反应机载折叠机械臂**的完整软硬件核心工程资产，涵盖 **STM32 下位机实时控制固件**、**PC 无线地面站上位机**、**YOLO26n-seg / OpenCV 黑色物块辅助视觉识别与往返标定程序**，以及 **SolidWorks 带实测核定质量参数的三维装配模型与动力学基线**。

---

## 一、仓库目录架构

```text
AerialRoboArm/
├── firmware_stm32/          # [1] 单片机程序：STM32F103C8T6 + FreeRTOS 双无线仲裁与舵机闭环固件
├── host_console_vision/     # [2] 上位机与视觉识别程序：PySide6 地面站 + YOLO26n-seg 分割定位与 LUT 标定
├── cad_solidworks_mass/     # [3] SolidWorks 带质量建模版：445.58 g 总装 (.SLDASM) + 41 个零件 (.SLDPRT) + 质量惯量 JSON/CSV
├── figures/                 # 系统硬件拓扑、软件架构图与实物样机照片
└── VIDEO_MANIFEST.md        # 实验与实机演示视频文件参数及未入库说明文档
```

---

## 二、系统核心设计特性

1. **单舵机类平行四边形大行程平动折展机构**：
   - 仅由 1 台 `HX8-U26H-M` 总线舵机同步驱动对称双平行连杆（孔心距 `210 mm`）实现机腹低矮折叠收纳与前向大跨度伸出（CAD 双向跨度 `395.31 mm`，实物全伸/收缩长 `870 / 460 mm`，展收比 `1.89`），折展全程末端相对安装基座保持水平指向；配合 `178 mm` 双侧长齿夹爪提供约 `95 mm` 单姿态水平覆盖容差。
2. **外啮合齿轮反向摆动与机载电控复用自平衡**：
   - 通过 `40:48:48`（`M = 0.8`）外啮合齿轮驱动多挡可调补偿臂等角反向摆动，直接复用机载必带的 STM32 主控电路板与锂电池充当反向配重（未加装专用配重块；CAD 仿真基线 `445.6 g`，含线束实测总质量 `506 g`）。
3. **独立供电控制、双无线安全仲裁与地面端辅助视觉**：
   - 机械臂自带独立供电与控制链路，无需读取或修改宿主无人机飞控；
   - **手动最高优先链路**：`2.4 GHz ELRS`（`USART1, CRSF 420 kbps`）提供实时遥控与急停接管；
   - **自动视觉与遥测链路**：`5.8 GHz` 模拟图传回传地面端执行 `YOLO26n-seg` 实例分割（掩膜二维质心 + 邻域暗色对比度门控，RTX 4070 单帧中位耗时 `12.55 ms`），经 `433 MHz HC13`（`USART3`）下发目标与回传 `HX8` 编码器 / `JY61P` 200 Hz 基座姿态加速度数据。

---

## 三、各模块快速开始

### 1. STM32 单片机程序（[`firmware_stm32/`](firmware_stm32/)）
- **硬件平台**：`STM32F103C8T6` + `FreeRTOS (CMSIS-RTOS v2)`
- **主要外设映射**：`USART1` → ELRS 接收机（实验固件切换为 `JY61P` 200 Hz IMU）、`USART2` → `UC-01` 总线舵机转接板（`HX8-U26H-M, 115200 bps`）、`USART3` → `HC13` 433 MHz 无线串口、`TIM2_CH1 (PA0)` → `PTK 7462W` 333 Hz PWM 夹爪舵机。
- **编译命令（需安装 `arm-none-eabi-gcc` 与 `CMake`）**：
  ```bash
  # 编译正式双无线控制固件
  cmake -S firmware_stm32 -B build/formal -G Ninja -DCMAKE_BUILD_TYPE=Release
  cmake --build build/formal

  # 编译台架动力学采集实验固件（HC13 V2 + HX8 + JY61P 200 Hz）
  cmake -S firmware_stm32 -B build/experiment -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DHX8_JY60_CAPTURE=ON -DJY61P_CAPTURE=ON -DHC13_HOST_LINK=ON
  cmake --build build/experiment
  ```
- 详细架构参见 [`firmware_stm32/Doc/design/CURRENT_SYSTEM.md`](firmware_stm32/Doc/design/CURRENT_SYSTEM.md) 与 [`firmware_stm32/README-zh.md`](firmware_stm32/README-zh.md)。

### 2. 上位机与视觉识别程序（[`host_console_vision/`](host_console_vision/)）
- **环境安装与启动（Python 3.10–3.12）**：
  ```bash
  cd host_console_vision
  pip install -r requirements.txt
  python main.py
  ```
- **内置模型与标定资产**：
  - 定制黑色物块分割权重：[`host_console_vision/assets/black_block_yolo26n_seg_rig_v1.pt`](host_console_vision/assets/black_block_yolo26n_seg_rig_v1.pt)（`6.22 MB`）
  - 换装加长夹爪后 `30°–70°` 连续标定查表：[`host_console_vision/assets/calibrated_position_lut.json`](host_console_vision/assets/calibrated_position_lut.json)
  - 运行单元测试：`pytest host_console_vision/tests`
- 详细操作与协议参见 [`host_console_vision/使用说明.md`](host_console_vision/%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.md) 与 [`host_console_vision/实验模式说明.md`](host_console_vision/%E5%AE%9E%E9%AA%8C%E6%A8%A1%E5%BC%8F%E8%AF%B4%E6%98%8E.md)。

### 3. SolidWorks 带质量参数三维模型（[`cad_solidworks_mass/`](cad_solidworks_mass/)）
- 使用 **SolidWorks 2022 及以上版本**直接打开 [`cad_solidworks_mass/悬挂点修订_IMU支架下扣_共享.SLDASM`](cad_solidworks_mass/%E6%82%AC%E6%8C%82%E7%82%B9%E4%BF%AE%E8%AE%A2_IMU%E6%94%AF%E6%9E%B6%E4%B8%8B%E6%89%A3_%E5%85%B1%E4%BA%AB.SLDASM) 即可查看完整三维装配与各零件质量属性。
- 多体运动学与动力学计算可直接读取 [`cad_solidworks_mass/质量参数_仿真基线_20261118/质量属性与装配坐标.json`](cad_solidworks_mass/%E8%B4%A8%E9%87%8F%E5%8F%82%E6%95%B0_%E4%BB%BF%E7%9C%9F%E5%9F%BA%E7%BA%BF_20261118/%E8%B4%A8%E9%87%8F%E5%B1%9E%E6%80%A7%E4%B8%8E%E8%A3%85%E9%85%8D%E5%9D%90%E6%A0%87.json) 与 [`零件质量明细.csv`](cad_solidworks_mass/%E8%B4%A8%E9%87%8F%E5%8F%82%E6%95%B0_%E4%BB%BF%E7%9C%9F%E5%9F%BA%E7%BA%BF_20261118/%E9%9B%B6%E4%BB%B6%E8%B4%A8%E9%87%8F%E6%98%8E%E7%BB%86.csv)。

---

## 四、实验录像与演示视频说明

原始标定采集录像（`sweep_20261123_200109.avi`，`146.67 MB`）及两段实机动作演示视频（`.mp4`）未直接存入 Git 版本库，其分辨率、帧率、采集工况及配套保留的同步标定数据请查阅 **[`VIDEO_MANIFEST.md`](VIDEO_MANIFEST.md)**。
