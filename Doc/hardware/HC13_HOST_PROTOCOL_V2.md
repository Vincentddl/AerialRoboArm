# HC13统一调试协议 V2（AerialRoboArm）

2026-10-30：本仓库已实现并完成ARM编译和软件互通。上位机为 `C:\Users\Vincent Lu\Downloads\desktop_app`，其 `v1_protocol.py/v1_state.py` 保留历史文件名，实际使用V2。**本次没有烧录或实物动作。**

与另一份103_a后端不同，本实现保留JY61P约200 Hz、HC13约20 Hz ACC、SERVO1独立调试和RECORD门控。ELRS不参与此构建。

## 传输和单位

- HC13连接USART3 PB10/PB11，230400、8N1；单个串口统一双向收发。
- ASCII英文逗号分隔、LF结束，接收兼容CRLF。有效载荷最多160字节，完整帧后解析；禁止空字段、非法数字和超限值。
- 位置及角度全部为mdeg，包括V帧；速度为°/s，时间为ms。未标定或无有效测量使用NA。
- INFO版本号为2，CAL包含14字段。V1不能直接连此构建，也不再接受V1整数度、speed=0和第五稳定帧字段。
- MCU样本接收时间不等于传感器内部采样时间或PC接收时间。ACC单位mg，包含重力。

## 当前配置

配置：`User/app/Inc/app_hc13_host_config.h`。

| 项目 | 程序配置 |
| --- | --- |
| HOME、NORMAL、PREP竖直位 | 同一HOST_VERTICAL_MDEG，当前编码器0° |
| 伸出方向和软件范围 | 负向，−90°～+85° |
| 最大速度 | 500°/s软件上限，非实测带载安全保证 |
| 普通MOVE/PICK加减速 | 各100 ms |
| 准备/折叠移动 | 30°/s，加减速各500 ms |
| SERVO1张开/闭合预设 | 135°/90°，只有PWM命令，没有实测夹持反馈 |
| 开爪、闭爪等待时间 | 用户确认没有实测值；配置−1，对外发布NA |
| 后折位置 | 未标定，发布NA，不猜测30° |
| 提前闭爪角 | 默认4°，PICK允许3～5°、0.1°步进 |
| 视觉候选置信阈值/期限 | 60%，500 ms |

未标定的准备、联动和折叠动作返回NOT_CALIBRATED，不填猜测等待时间。软件测试使用明确的模拟时间参数覆盖完整状态机，测试值不写入正式配置。

## PC→MCU

| 报文 | 作用 |
| --- | --- |
| `GET,id,INFO` / `GET,id,CAL` / `GET,id,STATE` | 只查询，不触发机构动作 |
| `PING,id` | 心跳，返回PONG |
| `MODE,id,MANUAL` / `VISION_DEMO` / `VISION_PICK` | 只改变路由，取消旧准备/候选/待闭爪流程；不自动回零或开爪 |
| `HOME,id` / `SHAPE,id,NORMAL` / `PREP,id` | 同一准备流程：低速到竖直位→实际反馈确认→开爪PWM→等待标定时间→READY→DONE |
| `SHAPE,id,FOLDED` | 先闭爪PWM→等待标定时间→向已标定位置后折→反馈确认→DONE |
| `PICK,id,target_mdeg,speed_dps,lead_mdeg` | READY后接受一次锁定参数的联动抓取 |
| `V,target_mdeg,speed_dps,confidence` | VISION_PICK且READY时只更新最新候选，不驱动HX8 |
| `TRIGGER,id` | 使用500 ms内的最新候选及默认提前量，触发一次PICK；消费候选 |
| `MOVE,id,target_mdeg,speed_dps` | 普通位置调试，不需要准备，不自动闭爪；仅MANUAL |
| `STOP,id` | 中止流程，取消所有待闭爪/候选/准备，尽可能保持HX8；已应用的夹爪PWM保持 |
| `GRIP,id,OPEN` / `CLOSE` | 独立夹爪命令；不等于夹持成功，执行后取消READY |
| `SERVO,id,1,angle_mdeg` | MANUAL下独立SERVO1角度调试，整度步进，执行后取消READY |
| `RECORD,id,1` / `0` | 实验记录门控，动作中拒绝切换；启用需有效200 Hz数据 |

MOVE/PICK角度范围−90000～85000，MOVE/PICK目标按100 mdeg步进，速度1～500。PICK的lead为3000～5000、按100 mdeg步进。SERVO角度0～180000，按1000 mdeg步进；仍受实际机构限制。

HOME/NORMAL仅MANUAL；PREP允许MANUAL和VISION_PICK；PICK允许MANUAL和VISION_PICK；TRIGGER仅VISION_PICK。VISION_DEMO不执行机构流程。独立GRIP仅在没有运行中的自动流程时执行。

## MCU→PC

```text
INFO,<id>,2,<boot_id>,<last_req_id>,230400,200,20,<max_speed>,<ramp_ms>,<post_ms>,<min_conf>,500
CAL,<id>,<vertical_mdeg>,<vertical_mdeg>,<extension_sign>,<max_extension_mdeg>,<folded_mdeg>,<open_mdeg>,<close_mdeg>,0,180000,<open_wait_ms>,<close_wait_ms>,<default_lead_mdeg>
ACC,<seq>,<sample_ms>,<ax_mg>,<ay_mg>,<az_mg>
STATE,<t_ms>,<mode>,<shape>,<motion>,<hx8_online>,<imu_online>,<pos_mdeg>,<target_mdeg>,<voltage_mv>,<imu_hz>,<record_on>,<acc_tx>,<grip_cmd_mdeg>,<grip_action>,<last_req_id>,<fault_code>,<reply_to>
ACK,<id>,<verb>,<result>
MOTION,<id>,<stage>,<t_ms>,<target_mdeg>,<pos_mdeg|NA>,<reason>
DONE,<id>,<verb>,<actual_pos_mdeg>
ERR,<id>,<code>
PONG,<id>,<t_ms>
```

STATE字段沿用V1的18字段定义，但主要按GET STATE回复，不主动高频广播；阶段变化用MOTION。ACC在MANUAL和VISION_DEMO约20 Hz，VISION_PICK停止无线传感器流，内部JY61P继续采样。

MOTION阶段：PREPARING、OPEN_APPLIED、READY、START、CLOSE_APPLIED、ARRIVED、END、ABORT。OPEN/CLOSE_APPLIED仅说明PWM已应用。READY先于准备DONE；任何STOP、MODE变化、故障、失联会取消READY及候选，迟到事件不得重新使能动作。

长动作ACK为ACCEPTED；独立PWM/STOP为APPLIED；MODE为实际模式名；RECORD为ON/OFF。只有DONE说明相应位置/准备流程完成，且仍不代表物体已被夹住。

正式默认标定回复示例：

```text
CAL,402,0,0,-1,90000,NA,135000,90000,0,180000,NA,NA,4000
```

## 单次联动判定

1. 必须完成准备、当前位置仍接近竖直位，有新鲜健康HX8反馈；需要的夹爪配置和等待时间必须已标定。
2. 目标必须在前伸软件范围内、沿配置方向前进；实际起点到目标行程必须大于提前量。
3. PICK/TRIGGER接受后锁定目标、速度和提前量，清除READY及候选；后续V不能改动本动作。
4. 只用新的feedback_seq和有效实际角度判断进度。允许采样跨过阈值，不要求恰好采到目标前4°。缓存同一帧不能推进闭爪条件。
5. 第一次跨过阈值后应用闭爪PWM并锁存，本动作最多一次。STOP/MODE/故障后的旧反馈不能迟到触发。
6. 实际到位、实验续录窗口及已配置闭爪等待均结束后才DONE。位置失败、准备失败或反馈过期只能ABORT/ERR。

PICK运动阶段向HX8请求5 ms轮询间隔，实际频率受串口5 ms帧间隔、回复耗时和硬件刷新率限制，不能宣称已实测200 Hz。到位仍保留约200 ms安静确认，避免高频查询把稳定判断缩短为几十毫秒。其他动作保持原50 ms轮询。

## 记录、故障和事务

- 保留JY61P约200 Hz与RTT完整记录；HC13的ACC只是20 Hz显示流，不是完整实验文件。RECORD ON/DONE不等于PC已经保存成功。
- 上电只初始化和查询，不自动回零、开爪、闭爪。TIM2只在明确PWM操作时启动。
- 请求序号匹配、内容冲突、旧请求拒绝与回复缓存保留；重复同帧不会重复移动、开爪或重置闭爪锁存。
- 关键MOTION事件单独缓存；无线发送背压不推迟本地角度阈值判定。READY/事件按序发出后再发DONE。
- 3秒没有有效PC消息会中止该无线会话的活动流程；只有查询的连接不会超时触发机构动作。
- 接收损坏后丢弃残帧并取消准备/候选。FAULT、FEEDBACK_STALE、MOTION_TIMEOUT等错误保留；底层TX_FAILED映射为新版解析器认识的FAULT。
- 折叠、开/闭爪等待仍未实测，默认NA是有意的阻止条件，不是通信失败。

## 验证与构建

```powershell
.\tools\test_hc13_host.ps1
.\tools\build_hc13_host.ps1
```

新版上位机互通测试使用 `tests/test_current_mcu_v2.py`，通过模拟串口连接实际C协议模块；HAL和执行器均模拟，没有运行实物。正式程序约FLASH35.4 KB、RAM13.8 KB。

详情见 [HC13_HOST_BUILD.md](./HC13_HOST_BUILD.md)。本次不烧录；后续先检查INFO/CAL/STATE/ACC，再标定等待时间与折叠位置，最后由用户明确启动动作。
