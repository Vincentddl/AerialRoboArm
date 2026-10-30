# 机械臂上位机通信协议 V1

日期：2026-10-30。用途：单片机、视觉程序和“机载机械臂实验上位机”的统一对接约定。

**本文件是完整接口约定，不代表当前固件已实现。当前板上采集程序仍通过RTT工作。本文不修改或烧录固件，不修改上位机代码。**

本文件替代 `HC13_HOST_PROTOCOL_DRAFT.md` 及此前聊天中的零散格式。报文均为格式示例，不是实时测量结果；标定值必须来自实际配置。

## 1. 连接和通用规则

| 项目 | V1约定 |
| --- | --- |
| 传输链路 | PC串口→电脑端HC13→板载HC13→STM32 USART3，反向同理 |
| 串口目标配置 | 230400、8N1，无流控；必须核对两块模块的实际配置，不能只改上位机下拉框 |
| 字符与分隔 | ASCII文本，英文逗号分隔，无空格、无空字段 |
| 帧结束 | 发送LF（`\n`），接收兼容CRLF（`\r\n`） |
| 最大帧长 | 160个有效载荷字节，不含行尾；超长丢弃到下一换行，禁止执行残缺指令 |
| 数字 | 十进制整数，不带单位字符；明确允许的缺失值使用大写`NA` |
| 时间 | uint32 MCU毫秒时间，重启归零；差值按uint32回绕处理 |
| 请求序号 | `req_id`为1～4294967295；单片机原样回传，用于匹配请求、去重和超时处理 |
| ACC序号 | 独立的uint32上报帧序号，每发一帧加一，不能与请求序号混用 |
| 实测反馈 | 没有收到反馈就显示未知，不能把已发送指令当作实测状态 |

串口接收必须处理粘包、分包：一次读取不等于一帧，只有遇到换行才解析。多余字段、非法字符、非法数字、字段数量错误均拒绝，不能用`atoi`失败后的0值执行动作。

电脑端只允许一个程序持有HC13串口。视觉程序和图形界面通过同一个串口管理器发送数据，不能各自打开同一COM口。

V1没有应用层额外CRC字段。接收端须严格校验帧类型、字段数、范围和请求序号；这些检查不能替代未来需要的端到端校验。

## 2. 单位

| 字段后缀 | 单位 | 示例 |
| --- | --- | --- |
| `_mg` | 0.001 g | `998`为0.998 g |
| `_mdeg` | 0.001° | `-29700`为−29.7° |
| `_mdps` | 0.001°/s | `12500`为12.5°/s |
| `_dps` | °/s | `250`为250°/s |
| `_mv` | mV | `11980`为11.98 V |
| `_ms` | ms | `152340`为MCU启动后152.340秒 |

加速度是JY61P输出的传感器数据，**包含重力**，不能标成已补偿的纯线加速度。小舵机没有实际角度反馈。

毫度是通信表示单位，不代表舵机具备0.001°定位精度。V1新发起的HX8目标角度按100 mdeg（0.1°）步进，小舵机角度按1000 mdeg（1°）步进，不符合步进的控制请求返回`OUT_OF_RANGE`。反馈仍按实际读数发送。

## 3. 工作模式与形态

模式和形态是两组独立状态。

| 工作模式 | 含义 | 无线ACC | 视觉目标可驱动HX8 |
| --- | --- | --- | --- |
| `MANUAL` | 手动实验和调试 | 开启，20 Hz | 否 |
| `VISION_DEMO` | 只展示图传及识别结果 | 开启，20 Hz | 否 |
| `VISION_PICK` | 视觉实时跟随、抓取 | 关闭 | 是 |

- 视觉演示页不提交机械臂运动命令。演示模式保留传感器回传，以便界面显示曲线。
- 抓取模式关闭ACC及可选GYRO/ATT无线流，保留必要的ACK/ERR和1 Hz状态帧。JY61P继续采样，RTT可继续本地记录。
- 进入抓取模式：停止传感器无线队列、清除旧视觉目标、应用已标定的夹爪张开PWM，再回复切换成功。标定缺失时返回`NOT_CALIBRATED`，不能假装已张开。
- 后续闭合由明确的`GRIP,CLOSE`请求触发；不恢复三帧后自动闭合。模式切换不自动发出出臂或回零指令。
- 手动出臂、回零、形态切换、SERVO角度调试只允许在MANUAL。GRIP开/闭允许在MANUAL和VISION_PICK。STOP适用于所有模式。
- 普通动作未结束时，进入另一模式返回`BUSY`。从VISION_PICK退出时允许取消持续视觉跟随并保持HX8当前位置，清除旧目标，再完成切换。

| 实际形态 | 含义 |
| --- | --- |
| `UNKNOWN` | 未标定、反馈不新鲜，或无法确认形态 |
| `NORMAL` | 已达到标定的正常形态 |
| `FOLDED` | 已达到标定的折叠形态 |

只有根据标定目标和实际反馈确认后才更新实际形态；最近请求和实际形态必须分别显示。`HOME`不自动等于`SHAPE,NORMAL`，是否同一位置由标定决定。

## 4. 单片机→上位机：数据帧

### 4.1 加速度，ACC

```text
ACC,<seq>,<sample_ms>,<ax_mg>,<ay_mg>,<az_mg>
```

```text
ACC,128,152340,85,-20,998
```

含义：无线帧128，MCU样本接收时间152340 ms，XYZ为0.085、−0.020、0.998 g。

- 加速度各轴有效范围按传感器量程−16000～16000 mg。
- `sample_ms`是该样本在MCU被解析的时间，不是电脑收包时间，也不是传感器内部采样时钟。
- 传感器约200 Hz，ACC无线流20 Hz取最新有效样本。没有新样本时不换时间戳重发旧值。
- 无线降频用于显示，不声称已做抗混叠滤波；完整200 Hz实验数据继续经RTT保存。
- IMU离线时停止ACC上报，STATE反映离线。上位机保留已有曲线并显示数据已过期，不持续补0。

### 4.2 状态，STATE

```text
STATE,<t_ms>,<mode>,<shape>,<motion>,<hx8_online>,<imu_online>,<pos_mdeg>,<target_mdeg>,<voltage_mv>,<imu_hz>,<record_on>,<acc_tx>,<grip_cmd_mdeg>,<grip_action>,<last_req_id>,<fault_code>,<reply_to>
```

```text
STATE,152340,MANUAL,UNKNOWN,IDLE,1,1,-29700,-30000,11980,200,1,1,120000,ANGLE_CMD,101,NONE,0
```

| 字段 | 类型/说明 |
| --- | --- |
| t_ms | 状态生成时MCU时间，不与ACC样本时间混用 |
| mode | MANUAL / VISION_DEMO / VISION_PICK |
| shape | UNKNOWN / NORMAL / FOLDED；是确认后的实际形态 |
| motion | IDLE / MOVING / SETTLING / FAULT；SETTLING包括到位后实验续录阶段 |
| hx8_online | 0或1，表示近期存在有效HX8反馈，不是“已达到目标” |
| imu_online | 0或1，表示近期存在有效IMU数据 |
| pos_mdeg | HX8实际角度；离线或无有效位置时NA |
| target_mdeg | 已接受并实际使用的HX8目标；尚无目标时NA；HX8离线不必抹掉已知目标 |
| voltage_mv | HX8反馈主电源电压；离线时NA |
| imu_hz | 实测ACC有效输出率；无有效数据为0，不能填无线显示频率20 |
| record_on | MCU实验记录门控，0或1；不等于电脑文件已经保存 |
| acc_tx | 当前ACC无线流是否启用，0或1；独立于record_on |
| grip_cmd_mdeg | SERVO1最近应用的命令角度，未设置时NA；不是实际反馈角度 |
| grip_action | UNKNOWN / OPEN_CMD / CLOSE_CMD / ANGLE_CMD，仅描述已应用的命令 |
| last_req_id | 最近处理的有副作用控制请求序号，无则0 |
| fault_code | NONE或第8节的错误码；有多项异常时上报当前主要原因，在线字段仍分别有效 |
| reply_to | GET STATE的请求序号；周期或事件触发的主动状态帧填0，与last_req_id区别开 |

STATE正常1 Hz；模式、运动、故障等关键变化可额外立即发送。超过3秒没有新状态/PONG，上位机显示链路离线，不能继续显示“实时在线”。

### 4.3 可选扩展

```text
GYRO,<seq>,<sample_ms>,<gx_mdps>,<gy_mdps>,<gz_mdps>
ATT,<seq>,<sample_ms>,<roll_mdeg>,<pitch_mdeg>,<yaw_mdeg>
```

各数据类型维护独立序号及样本时间。首版默认只开启ACC，GYRO/ATT为可选能力，不默认把全部200 Hz数据塞入无线链路。六轴姿态模块的yaw不应显示为有绝对航向保证。

## 5. 上位机→单片机：控制指令

### 5.1 完整指令表

| 指令格式 | 参数和用途 |
| --- | --- |
| `MOVE,<req_id>,<target_mdeg>,<speed_dps>` | HX8绝对编码器角度目标；−90000～85000 mdeg；速度1～500°/s，受INFO/CAL实际范围进一步限制 |
| `HOME,<req_id>` | 按CAL公布的home_mdeg回零；速度30°/s、加减速各500 ms；不计作普通高速出臂实验 |
| `STOP,<req_id>` | 取消视觉跟随/当前自动流程，HX8保持当前位置；夹爪保持当前命令角度 |
| `MODE,<req_id>,MANUAL` | 切换手动模式 |
| `MODE,<req_id>,VISION_DEMO` | 切换仅视觉展示模式 |
| `MODE,<req_id>,VISION_PICK` | 切换视觉抓取模式，停止无线传感器流并应用夹爪张开命令 |
| `SHAPE,<req_id>,FOLDED` | 按已标定的折叠流程执行；未知标定返回NOT_CALIBRATED |
| `SHAPE,<req_id>,NORMAL` | 按已标定的正常形态流程执行；不能只把界面状态改成NORMAL |
| `GRIP,<req_id>,OPEN` | SERVO1应用已标定的张开角度 |
| `GRIP,<req_id>,CLOSE` | SERVO1应用已标定的闭合角度 |
| `SERVO,<req_id>,1,<angle_mdeg>` | SERVO1独立角度调试，0～180000 mdeg、1000 mdeg步进；仍受实际标定范围限制；V1不接受其他通道号 |
| `RECORD,<req_id>,1` | 开启MCU完整实验记录门控，不开启视觉模式的无线ACC流 |
| `RECORD,<req_id>,0` | 关闭记录门控；动作/续录未结束时返回BUSY，不能截断当前记录 |
| `GET,<req_id>,STATE` | 查询一次状态 |
| `GET,<req_id>,INFO` | 查询协议版本、通信及运动参数 |
| `GET,<req_id>,CAL` | 查询角度方向、形态和夹爪标定 |
| `PING,<req_id>` | 心跳查询 |

普通MOVE加减速默认各100 ms，开启记录后默认到位再续录1000 ms；实际参数以INFO为准。MOVE的ACK不能绕过反馈新鲜度、供电、限位、故障等现有检查。

HOME、SHAPE、GRIP都不能暗中执行未配置的标定动作。SHAPE默认以30°/s、加减速各500 ms执行已标定的形态动作；若该构建无法确认机构形态，须返回UNSUPPORTED或保持UNKNOWN，不能以请求成功代替实测完成。

STOP是控制器保持请求，不是硬件断电急停；HX8失联时应报告执行保持未确认，同时仍取消自动目标，不能回报已保持成功。在VISION_PICK中执行STOP时同时切回MANUAL，STATE立即报告新模式，随后残留V帧不得重启运动；重新视觉跟随需要新的MODE请求。

### 5.2 速度和角度的界面换算

- 速度滑块为百分比时：`speed_dps = round(INFO.max_speed_dps × 百分比 / 100)`。0%禁止启动；正百分比计算后至少1°/s。
- 例如最高速度500°/s，50%发送250°/s，而不是发送数字50。
- “出臂角度”若显示正的伸出量：`target_mdeg = CAL.normal_mdeg + CAL.extension_sign × 出臂角度° × 1000`。
- 若normal=0、extension_sign=−1，则界面出臂30°对应编码器−30000 mdeg。
- 未取得有效CAL时，界面可以显示输入，但不能猜测正负方向或折叠角度后直接发MOVE。
- 只点击“启动出臂”才发送MOVE；拖动滑块仅改变待发送参数。

示例：

```text
MOVE,101,-30000,250
SERVO,102,1,120000
GRIP,103,OPEN
GRIP,104,CLOSE
STOP,105
```

## 6. 视觉目标兼容格式

保留现有视觉程序的低延迟格式：

```text
V,<angle_deg>,<speed_dps>,<confidence>
```

```text
V,-60,500,95
```

注意：这里角度为**整数度**，与MOVE/STATE的毫度不同。confidence为0～100。只在VISION_PICK接收为控制目标；MANUAL和VISION_DEMO不得由V帧驱动机械臂。

原解析器的可选第五字段stable_frames可继续兼容，但不得据此自动闭合夹爪。有效视觉目标可以逐帧更新，不增加三帧或五帧等待。

视觉帧不逐帧ACK，避免占满半双工链路；工作状态及拒绝原因低频汇总到STATE。超出角度/速度范围、低于实际配置置信阈值或数据过期的帧不能驱动舵机。超过300 ms没有新的有效视觉帧，清除旧目标并保持HX8，不自动回零、归位或闭合。

相机图像和目标像素坐标由电脑视觉源直接交给界面，**视频不经过HC13**；HC13只承载目标控制和少量状态。

## 7. 执行回复：接受、设置、完成分开

### 7.1 ACK

```text
ACK,<req_id>,<verb>,<result>
```

| 回复示例 | 解释 |
| --- | --- |
| `ACK,101,MOVE,ACCEPTED` | 已检查并启动MOVE，不代表已到位 |
| `ACK,106,HOME,ACCEPTED` | 已启动回零 |
| `ACK,107,SHAPE,ACCEPTED` | 已启动已标定形态流程 |
| `ACK,108,MODE,VISION_PICK` | 模式已实际切换，传感器无线队列已停，夹爪张开PWM已应用 |
| `ACK,109,GRIP,APPLIED` | PWM张开/闭合命令已应用，不代表实物完成动作 |
| `ACK,110,SERVO,APPLIED` | 调试角度PWM已应用 |
| `ACK,111,RECORD,ON` | MCU记录门控已开启，不代表电脑已开始存盘 |
| `ACK,112,RECORD,OFF` | 门控已关闭，当前不存在被截断的动作记录 |
| `ACK,113,STOP,APPLIED` | 已取消自动流程并成功下发保持命令；夹爪设置未改变 |

### 7.2 DONE

```text
DONE,<req_id>,<verb>,<actual_pos_mdeg>
```

```text
DONE,101,MOVE,-29700
```

- 只用于需要实际HX8位置/流程确认的MOVE、HOME、SHAPE。到位使用固件的误差、静止与反馈新鲜度判定。
- 若record_on=1，DONE在到位后的续录窗口结束后发送；ARRIVED事件可提前说明到位。
- 普通MOVE当前到位容差1°；低速HOME当前容差0.5°，须保留连续安静反馈判断。协议单位不是定位精度保证。
- 小舵机没有位置或夹持反馈，不能发送声称实物到位/夹住物体的DONE。它只能ACK为APPLIED，STATE上报命令角度。

### 7.3 MOTION动作事件

```text
MOTION,<req_id>,<stage>,<t_ms>,<target_mdeg>,<pos_mdeg>,<reason>
```

```text
MOTION,101,START,152100,-30000,0,NONE
MOTION,101,ARRIVED,152620,-30000,-29700,NONE
MOTION,101,END,153620,-30000,-29700,NONE
DONE,101,MOVE,-29700
```

stage为START / ARRIVED / END / ABORT。START时间对应发出舵机位置指令；ARRIVED对应程序判定到位；END对应本动作和记录窗口完成。若故障或STOP中止，发送ABORT和相应原因，不发送成功DONE。不能将MOTION事件直接称为200 Hz文件保存成功。

## 8. 错误回复

```text
ERR,<req_id>,<code>
```

无法解析请求序号时使用req_id=0；字段错误时不能复用上一次有效目标。

| code | 含义 |
| --- | --- |
| BAD_FRAME | 类型、字段数、字符或帧结构错误 |
| BAD_NUMBER | 数字缺失、非法、溢出 |
| OUT_OF_RANGE | 角度、速度、步进或通道超范围 |
| NOT_READY | 当前未满足启动条件 |
| BUSY | 当前有未结束动作或记录窗口 |
| MODE_DENIED | 当前模式不允许该控制 |
| NOT_CALIBRATED | 所需零位、形态或夹爪角度未标定 |
| HX8_OFFLINE | 没有新鲜的有效舵机反馈 |
| IMU_OFFLINE | 没有新鲜IMU数据 |
| IMU_RATE_LOW | 需要200 Hz实验记录，但实测速率未达要求 |
| FAULT | 当前硬件/控制故障，详细状态由STATE提供 |
| MOTION_TIMEOUT | 动作未在规定时间内满足完成条件 |
| FEEDBACK_STALE | 动作期间舵机反馈过期 |
| VISION_STALE | 视觉目标过期，已停止持续跟随 |
| LINK_LOST | 无线控制会话通信过期 |
| STOPPED | 动作被明确停止 |
| REQUEST_CONFLICT | 同一请求序号对应不同控制内容 |
| STALE_REQUEST | 请求早于当前已处理控制序号，且不在可重放缓存中 |
| UNSUPPORTED | 该构建未包含所请求功能，不能回复成功 |

收到ACK后动作途中发生故障，应使用该动作的req_id发送ERR和MOTION,ABORT，最终状态为FAULT或IDLE保持；不能用DONE掩盖中止。

## 9. 查询与心跳

### 9.1 INFO

```text
GET,401,INFO
INFO,<req_id>,<version>,<boot_id>,<last_req_id>,<uart_baud>,<imu_target_hz>,<acc_tx_hz>,<max_speed_dps>,<ramp_ms>,<post_ms>,<vision_min_confidence>,<vision_ttl_ms>
```

```text
INFO,401,1,73582,113,230400,200,20,500,100,1000,80,300
```

boot_id为当前设备启动标识，设备重启应改变；上位机发现变化后开启新会话并清除旧请求匹配。`last_req_id`只统计有副作用的控制，不统计GET/PING。置信阈值80只是示例，固件必须返回实际配置值。

### 9.2 CAL

```text
GET,402,CAL
CAL,<req_id>,<home_mdeg>,<normal_mdeg>,<extension_sign>,<max_extension_mdeg>,<folded_mdeg>,<grip_open_mdeg>,<grip_close_mdeg>,<servo1_min_mdeg>,<servo1_max_mdeg>
```

```text
CAL,402,0,0,-1,90000,NA,NA,NA,0,180000
```

- 未标定字段发NA，extension_sign只允许−1、1或NA。
- 示例代表折叠及夹爪开闭位置未知，不能据此允许这些流程；不自动假定90°/130°。
- home和normal是独立配置，可相同但不能由上位机猜测。
- 第一次真实接入必须先读CAL；标定写入不纳入V1远程控制指令，避免运行时误改方向/零点。

### 9.3 PING/PONG

```text
PING,403
PONG,403,152350
```

PONG字段为请求序号和MCU当前时间。上位机建议每秒PING一次；三秒无有效状态/PONG即标记离线。

无线控制会话启用期间，MCU若连续三秒没有收到有效上位机消息，应取消该会话的持续自动控制并尝试保持HX8；不能自动回零或张开夹爪。此策略只作用于无线控制所有者，不应在尚未建立无线控制会话时干扰独立RTT实验。

## 10. 去重、重试和超时

- 对MOVE/HOME/STOP/MODE/SHAPE/GRIP/SERVO/RECORD，上位机使用当前boot_id会话内递增且非零的req_id。
- 设备缓存近期请求内容和最新结果。同req_id同内容只重发当前结果，不再次执行；同req_id不同内容返回REQUEST_CONFLICT。
- 缓存已淘汰的旧控制序号返回STALE_REQUEST，不能重新当作新动作执行。上位机重连后通过INFO获知last_req_id，从更大的序号继续。
- GET/PING不执行机构动作，只按req_id回送查询，不受控制序号水位限制，保证重连时能查询。
- 没有收到ACK不等于设备未执行。只能用相同req_id、相同内容重试，不能为同一个点击重新生成新序号。
- 首版ACK超时建议500 ms、最多重试2次；这是联调起点，不是链路延迟保证。失败后显示结果未确认并查询状态，不能默认为成功。
- 动作完成以DONE/ERR/MOTION事件为准，不能把ACK超时当作动作结束。普通HX8移动超时沿用当前10秒；HOME等特殊流程依实现设定，超时必须回报。
- STOP优先于普通队列，不因BUSY或视觉模式而延迟。不要用关闭电脑串口代替STOP。

## 11. 与实验记录、视觉流程的关系

- `record_on`控制MCU实验门控；`acc_tx`控制低频无线显示，两者独立。
- 当前实验窗口为命令前约200 ms，加上动作过程和确认到位后1000 ms。完整200 Hz数据由RTT采集程序保存。
- 上位机仅收到无线20 Hz ACC时，只能保存20 Hz显示数据，不能声称已保存完整200 Hz实验。
- MCU的DONE只证明控制和自身记录窗口完成。上位机若要显示“文件已保存、COMPLETE=True”，需要接收电脑端完整采集程序的保存结果。
- 进入VISION_PICK不停止传感器供电或200 Hz采样。模式切换停止的是无线传感器流，离开抓取模式再恢复。
- 视觉目标发送前，等待MODE的成功ACK。切换期间不得排队发送旧目标；离开抓取模式后忽略残留V帧。

## 12. 界面控件到指令的对应

| 界面操作 | 发送/显示规则 |
| --- | --- |
| 连接 | 打开串口→GET INFO→GET CAL→GET STATE→开始心跳；只打开COM口不算机械臂在线 |
| 折叠模式/正常模式 | SHAPE；ACK显示执行中，DONE及STATE确认后更新实际形态 |
| 三个工作模式按钮 | MODE；收到成功ACK后更新当前模式，最近请求单独显示 |
| 拖动速度/角度 | 只更新待发送参数 |
| 启动出臂 | 换算CAL和INFO后发一次MOVE |
| 归零/恢复位置 | HOME，使用已配置零位 |
| 夹爪打开/闭合 | GRIP，显示“张开/闭合命令已设置”，没有反馈就不显示“已夹住” |
| 小舵机角度调试 | SERVO通道1；仅MANUAL，显示命令角度 |
| 停止 | STOP，取消流程；不自动松开夹爪 |
| 暂停显示/清空曲线/最近N秒 | 电脑本地操作，不发送电机命令，不改变采样或record_on |
| 模拟演示 | 与真实串口明确互斥，模拟帧不得驱动真实舵机 |
| XYZ曲线 | ACC的mg÷1000，以样本MCU时间绘图；电脑接收时间单独记录 |
| 视觉图像 | 从本地视觉源接入，不通过HC13传视频 |

## 13. 一次完整的手动联调示例

以下假设设备功能已实现、标定有效、MANUAL且IDLE；每行实际发送时追加LF。

```text
GET,401,INFO
GET,402,CAL
GET,403,STATE
PING,404
RECORD,501,1
ACK,501,RECORD,ON
MOVE,502,-30000,250
ACK,502,MOVE,ACCEPTED
MOTION,502,START,152100,-30000,0,NONE
ACC,128,152340,85,-20,998
MOTION,502,ARRIVED,152620,-30000,-29700,NONE
MOTION,502,END,153620,-30000,-29700,NONE
DONE,502,MOVE,-29700
GRIP,503,OPEN
ACK,503,GRIP,APPLIED
SERVO,504,1,120000
ACK,504,SERVO,APPLIED
MODE,505,VISION_PICK
ACK,505,MODE,VISION_PICK
V,-60,500,95
GRIP,506,CLOSE
ACK,506,GRIP,APPLIED
STOP,507
ACK,507,STOP,APPLIED
MODE,508,MANUAL
ACK,508,MODE,MANUAL
```

MODE进入VISION_PICK成功后不得再发ACC/GYRO/ATT；STATE继续。停止后不会自动恢复视觉运动，需新的明确控制流程。所有命令的成功示例都不能代替实际接入验证。

## 14. 实现边界

本协议定义了完整对接格式和状态语义；**当前尚未在MCU或上位机中实现**。原有RTT `g/v/r/h/x` 以及现有完整固件的视觉解析行为不能冒充本V1协议。

后续接入顺序：先只验证ACC/STATE和GET/PING，再验证请求去重和模式互斥，之后接入已标定的机构动作。不得仅凭本文示例把未知折叠或夹爪位置写成固定值。
