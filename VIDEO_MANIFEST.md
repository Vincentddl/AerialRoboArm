# 项目实验与演示视频文件说明文档（Video & Recording Manifest）

为保持 GitHub 仓库克隆轻量并避免超过单文件体积上限，本项目**不向 Git 仓库提交 `.avi` / `.mp4` 等原始视频文件**。所有涉及的实机演示视频与视觉标定原始录像均在此登记其技术参数、实验工况及仓库内配套保留的数据文件。

---

## 1. 未上传视频文件汇总表

| 序号 | 原始视频文件名 | 原属模块路径 | 文件大小 | 分辨率与帧率 | 帧数 / 时长 | 实验 / 演示内容概述 |
| :---: | :--- | :--- | :---: | :---: | :---: | :--- |
| 1 | `sweep_20261123_200109.avi` | `host_console_vision/runtime/position_recordings/sweep_20261123_200109/` | `146.67 MB` | `640 × 480` @ `30.0 fps` | `8798 帧` (`293.3 s`) | **30°–70° 步进往返标定与 5.8 GHz 模拟图传黑块分割实采录像**：换装加长双侧长齿夹爪后，按 `5°` 步长完成去程 9 点 + 回程 8 点共 17 次驻留（每点驻留 3 s，共 1559 帧驻留画面），用于训练/验证 `YOLO26n-seg` 黑色物块掩膜质心提取并生成连续查表 LUT。 |
| 2 | `基于 ELRS 遥控器的手动闭环控制演示.mp4` | `figures/DEMO/` | `20.81 MB` | `1920 × 1080` @ `30.0 fps` | `287 帧` (`9.6 s`) | **2.4 GHz ELRS 手动链路实机闭环演示**：通过 `USART1`（`CRSF, 420 kbps`）实时遥控 HX8-U26H-M 主驱动舵机与 PTK 7462W 夹爪舵机，验证摇杆死区处理、模式切换与急停（`SD`）最高优先级安全接管。 |
| 3 | `实机闭环控制折叠臂模拟动作演示.mp4` | `figures/DEMO/` | `2.43 MB` | `1280 × 720` @ `30.1 fps` | `264 帧` (`8.8 s`) | **折叠机构大行程平动与反向动平衡联动实机演示**：展示单舵机驱动类平行四边形连杆前伸/后折全过程，验证末端夹爪相对基座水平指向保持及 40:48:48 外啮合齿轮驱动电控补偿臂等角反向同步摆动。 |

---

## 2. 无原始视频条件下的实验复现说明

虽然原始 `.avi` / `.mp4` 录像未纳入仓库，但仓库中**完整保留了由上述实验生成的全部结构化同步数据与模型权重**，可直接运行上位机与单元测试复现核心结果：

1. **同步角度事件流**：[`host_console_vision/runtime/position_recordings/sweep_20261123_200109/sweep_20261123_200109.angles.jsonl`](host_console_vision/runtime/position_recordings/sweep_20261123_200109/sweep_20261123_200109.angles.jsonl) 完整记录了 17 次驻留期间的上位机目标角与 HX8 12 位磁编码器实测回读角。
2. **实测连续标定查表（LUT）**：
   - [`host_console_vision/assets/calibrated_position_lut.json`](host_console_vision/assets/calibrated_position_lut.json) 与 [`position_lut_yolo26.json`](host_console_vision/runtime/position_recordings/sweep_20261123_200109/position_lut_yolo26.json) 保存了 `30°–70°` 共 9 个设定角的掩膜质心纵坐标均值（`369.18 → 156.87 px`，斜率约 `−5.4 px/°`）、8 组配对角编码器去回程回差（均值 `0.53°`、最大 `0.80°`）及像素重复差（均值 `1.96 px`）。
3. **定制分割模型权重**：[`host_console_vision/assets/black_block_yolo26n_seg_rig_v1.pt`](host_console_vision/assets/black_block_yolo26n_seg_rig_v1.pt)（`6.22 MB`）已随库提供，接入任意 UVC 图传接收机或 USB 摄像头即可直接在上位机中运行实时分割识别与掩膜质心提取（无需原始标定录像）。
