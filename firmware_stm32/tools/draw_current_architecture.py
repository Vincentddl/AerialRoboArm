"""Render source-grounded architecture figures as editable SVG and PNG."""
from pathlib import Path
from html import escape
import math
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "Figures" / "architecture"
BLUE, TEAL, INK, MUTED = "#315680", "#287969", "#203044", "#596979"


class Figure:
    def __init__(self, height=1500):
        self.w, self.h = 2040, height
        self.im = Image.new("RGB", (self.w, self.h), "white")
        self.d = ImageDraw.Draw(self.im)
        self.svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="2040" height="{height}" viewBox="0 0 2040 {height}">', '<rect width="100%" height="100%" fill="white"/>']

    def text(self, x, y, value, size=28, color=INK, bold=False, center=True):
        font = ImageFont.truetype("C:/Windows/Fonts/msyhbd.ttc" if bold else "C:/Windows/Fonts/msyh.ttc", size)
        self.d.text((x, y), value, fill=color, font=font, anchor="mm" if center else "lm")
        self.svg.append(f'<text x="{x}" y="{y}" text-anchor="{"middle" if center else "start"}" dominant-baseline="central" font-family="Microsoft YaHei, sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{color}">{escape(value)}</text>')

    def rect(self, x, y, w, h, fill="#f4f7fb", stroke=BLUE, radius=10):
        self.d.rounded_rectangle((x, y, x+w, y+h), radius, fill=fill, outline=stroke, width=2)
        self.svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')

    def box(self, x, y, w, h, title, lines=(), fill="#f4f7fb", color=BLUE, title_size=30, size=25):
        self.rect(x, y, w, h, fill, color)
        count = 1 + len(lines)
        step = 38
        yy = y+h/2-(count-1)*step/2
        self.text(x+w/2, yy, title, title_size, color, True)
        for line in lines:
            yy += step
            self.text(x+w/2, yy, line, size)

    def arrow(self, points, color=BLUE, dashed=False, width=3):
        if dashed:
            for p, q in zip(points, points[1:]):
                dist = math.dist(p, q)
                for start in range(0, int(dist), 17):
                    stop = min(start+10, dist)
                    self.d.line([(p[0]+(q[0]-p[0])*t/dist, p[1]+(q[1]-p[1])*t/dist) for t in (start, stop)], fill=color, width=width)
        else:
            self.d.line(points, fill=color, width=width, joint="curve")
        p, q = points[-2:]
        angle = math.atan2(q[1]-p[1], q[0]-p[0])
        head = [q, (q[0]-14*math.cos(angle-.45), q[1]-14*math.sin(angle-.45)), (q[0]-14*math.cos(angle+.45), q[1]-14*math.sin(angle+.45))]
        self.d.polygon(head, fill=color)
        coords = " ".join(f"{x},{y}" for x, y in points)
        dash = ' stroke-dasharray="10 7"' if dashed else ""
        self.svg.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"{dash}/><polygon points="{" ".join(f"{x},{y}" for x,y in head)}" fill="{color}"/>')

    def save(self, name):
        OUT.mkdir(parents=True, exist_ok=True)
        self.im.save(OUT / f"{name}.png")
        (OUT / f"{name}.svg").write_text("\n".join(self.svg+["</svg>"]), encoding="utf-8")
        print(OUT / f"{name}.png")


def overall():
    f = Figure(1540)
    f.text(1020, 62, "机械臂控制系统总体架构", 46, bold=True)
    f.text(1020, 118, "正式控制构建 · 电脑视觉处理 / STM32 监督控制 / 舵机内部闭环", 27, MUTED)
    f.box(70, 190, 400, 120, "机载相机", ["采集目标图像"])
    f.box(550, 190, 400, 120, "模拟图传与视频采集", ["接收视频并输入电脑"])
    f.box(1030, 190, 450, 120, "电脑视觉处理", ["目标识别 · 标定与角度映射"])
    f.box(1560, 190, 410, 120, "HC13 无线数据链路", ["电脑端 / 机载端"])
    for left, right in [(470, 550), (950, 1030), (1480, 1560)]:
        f.arrow([(left, 250), (right, 250)])
    f.text(1020, 352, "视频通过图传进入电脑；HC13 传输目标参数与接收确认，不传输视频", 25, MUTED)
    f.arrow([(1765, 310), (1765, 385), (1120, 385), (1120, 450)])
    f.text(1365, 409, "目标角度 / 速度 / 置信度 · USART3", 23, BLUE)

    f.text(270, 427, "操作与测距", 30, bold=True)
    f.text(1770, 427, "执行机构", 30, bold=True)
    f.box(70, 470, 400, 120, "ELRS 遥控器", ["手动操作 · 模式 · 急停 / 复位"])
    f.box(70, 650, 400, 120, "ELRS 接收器", ["输出 CRSF 通道数据"])
    f.arrow([(270, 590), (270, 650)])
    f.text(350, 619, "无线遥控", 23, MUTED)
    f.arrow([(470, 710), (650, 710)])
    f.text(560, 666, "CRSF", 24, BLUE)
    f.text(560, 699, "USART1", 23, BLUE)
    f.box(70, 970, 400, 160, "VL53L1X 测距", ["距离 / 有效性 / 窗口判定", "独立采样与快照"], color=TEAL, fill="#f0f8f5")
    f.arrow([(470, 1080), (650, 1080)], TEAL, True)
    f.text(560, 1046, "I²C1", 24, TEAL)

    f.rect(650, 450, 760, 750, "#ffffff", BLUE, 14)
    f.text(1030, 490, "STM32F103C8T6 · FreeRTOS", 33, BLUE, True)
    f.box(690, 540, 680, 110, "输入解析与目标校验", ["遥控通道解析 · 视觉帧解析与时效检查"])
    f.box(690, 700, 680, 125, "模式选择与安全仲裁", ["MANUAL / AUTO · 急停 / 失联 / 故障锁定"])
    f.box(690, 875, 680, 130, "机械臂 / 夹爪状态机与执行监督", ["目标生成 · 限幅 · 反馈检查与位置微调"])
    f.arrow([(1030, 650), (1030, 700)])
    f.arrow([(1030, 825), (1030, 875)])
    f.box(690, 1050, 680, 90, "独立测距任务与快照", ["当前不作为完整抓取闭环的已启用依据"], color=TEAL, fill="#f0f8f5", title_size=27, size=23)
    f.text(1030, 1170, "ControlTask：20 ms 调度；HX8 反馈查询：100 ms", 23, MUTED)

    f.box(1570, 470, 400, 100, "UC-01 总线转接")
    f.box(1570, 650, 400, 210, "HX8-U26H-M 主臂舵机", ["内部位置闭环", "驱动机械臂折展", "回传角度与电气状态"], title_size=28)
    f.arrow([(1410, 935), (1470, 935), (1470, 520), (1570, 520)])
    f.text(1490, 477, "USART2", 23, BLUE)
    f.arrow([(1770, 570), (1770, 650)])
    f.text(1862, 611, "舵机总线", 23, BLUE)
    f.arrow([(1570, 780), (1510, 780), (1510, 810), (1410, 810)], TEAL, True)
    f.text(1510, 861, "角度 / 状态", 22, TEAL)
    f.text(1510, 891, "经 UC-01 返回", 20, TEAL)
    f.box(1570, 990, 400, 140, "PWM 夹爪舵机", ["执行开爪 / 闭爪", "无实际位置 / 夹持反馈"])
    f.arrow([(1410, 1018), (1570, 1018)])
    f.text(1490, 966, "TIM2_CH1", 22, BLUE)
    f.text(1490, 996, "PWM", 22, BLUE)

    f.box(650, 1300, 760, 100, "电脑调试与运行记录", ["DAPLink / OpenOCD / SEGGER RTT"], color=MUTED, fill="#f7f8fa")
    f.arrow([(1030, 1200), (1030, 1300)], MUTED, True)
    f.text(1155, 1247, "SWD / RTT", 24, MUTED)
    f.arrow([(75, 1270), (140, 1270)], BLUE)
    f.text(155, 1270, "图像 / 目标 / 控制", 23, center=False)
    f.arrow([(75, 1320), (140, 1320)], TEAL, True)
    f.text(155, 1320, "测量 / 状态反馈", 23, center=False)
    f.text(1020, 1452, "JY61P 属于独立采集构建，与 ELRS 复用 USART1；不画为正式控制中的并行输入。", 25, MUTED)
    f.text(1020, 1497, "图示依据当前仓库实现；电脑视觉源码在外部项目，系统性能以实测为准。", 23, MUTED)
    f.save("control_system_current_20261114")


def software():
    f = Figure(1470)
    f.text(1020, 62, "机械臂控制系统软件分层架构", 46, bold=True)
    f.text(1020, 119, "正式 FreeRTOS 构建 · 任务编排与顺序控制函数分开表达", 27, MUTED)
    layers = [
        (190, 145, "APP", "应用任务与调度", ["ControlTask 20 ms  ·  DebugTask 100 ms", "TofTask 独立采样  ·  defaultTask 维护与条件喂狗"]),
        (395, 160, "TASK", "控制步骤与状态机", ["TaskRc → TaskVision → TaskArbiter → TaskManipulator → TaskMotion", "以上为 ControlTask 顺序调用的功能步骤，并非五个独立线程"]),
        (615, 145, "MOD", "控制支撑模块", ["mod_rc_semantic  ·  mod_startup_angle  ·  mod_home_zero", "mod_position_trim  ·  调试请求处理"]),
        (820, 145, "DRV", "设备驱动与通信协议", ["drv_elrs  ·  drv_hc13  ·  drv_fsus  ·  drv_servo", "drv_vl53l1x / VL53L1X_api  ·  dev_status"]),
        (1025, 120, "BSP", "板级外设抽象", ["bsp_uart  ·  bsp_pwm  ·  bsp_i2c  ·  bsp_gpio"]),
    ]
    for y, h, tag, title, lines in layers:
        f.rect(70, y, 1510, h, "#ffffff", BLUE)
        f.text(180, y+h/2, tag, 33, BLUE, True)
        f.box(295, y+15, 1265, h-30, title, lines, title_size=28, size=24)
    for (_, y1), (_, y2) in zip([(0,y+h) for y,h,*_ in layers], [(0,y) for y,*_ in layers][1:]):
        f.arrow([(830, y1), (830, y2)])
    f.box(70, 1210, 1510, 110, "运行基础", ["STM32 HAL / CMSIS  ·  FreeRTOS  ·  UART / DMA / I²C / TIM / IWDG / SWD"], title_size=29, size=25)
    f.arrow([(830, 1145), (830, 1210)])
    f.box(1650, 395, 320, 340, "DataHub", ["ControlTask 单写者", "多读者状态快照", "模式 / 目标 / 反馈", "不承担实时控制链传递"], title_size=31, size=23)
    f.arrow([(1580, 470), (1650, 470)], TEAL, True)
    f.text(1775, 779, "调试与观测读取快照", 23, TEAL)
    f.box(1650, 855, 320, 190, "测距独立快照", ["AppTofSnapshot", "独立于 DataHub", "当前不直接驱动夹爪"], color=TEAL, fill="#f0f8f5", title_size=28, size=23)
    f.text(1020, 1390, "已移除旧图中的 alg_pid / alg_voltage_foc / drv_as5600 / drv_bldc_power；主臂闭环由 HX8 内部完成。", 25, MUTED)
    f.text(1020, 1435, "JY61P 与 HC13 V2 实验采集路径单独构建，不并入本图的正式任务结构。", 24, MUTED)
    f.save("software_architecture_current_20261114")


def five_columns():
    """Keep the user's five vertical panels and cylindrical DataHub shape."""
    f = Figure(990)
    f.text(1020, 43, "机械臂控制系统总体分层架构", 35, "#111111", True)

    def panel(x, y, w, h, label):
        f.rect(x, y, w, h, "white", "#111111", 0)
        f.text(x+w/2, y+26, label, 26, "#111111", True)

    def item(x, y, w, h, title, lines=(), size=24):
        f.rect(x, y, w, h, "white", "#111111", 0)
        yy = y+h/2-len(lines)*17
        f.text(x+w/2, yy, title, size, "#111111")
        for line in lines:
            yy += 34
            f.text(x+w/2, yy, line, 21, "#111111")

    panel(20, 195, 290, 600, "执行与反馈层")
    item(45, 265, 240, 110, "HX8 主臂舵机", ["舵机内部位置闭环"])
    item(45, 460, 240, 90, "夹爪 PWM 舵机")
    item(45, 640, 240, 100, "VL53L1X 测距", ["独立测量快照"])

    panel(365, 195, 395, 600, "算法与驱动层")
    item(400, 265, 325, 85, "目标限幅与位置微调")
    item(400, 395, 325, 85, "HX8 通信与反馈读取", ["USART2 / UC-01"], 23)
    item(400, 525, 325, 85, "夹爪舵机驱动", ["TIM2 PWM"])
    item(400, 655, 325, 85, "测距传感器驱动", ["I²C / VL53L1X"])

    panel(815, 100, 330, 790, "功能任务层")
    item(850, 170, 260, 80, "遥控解析")
    item(850, 310, 260, 80, "视觉目标接收与校验", size=22)
    item(845, 450, 270, 85, "机械臂行为决策", ["主臂 / 夹爪状态机"])
    item(850, 590, 260, 85, "运动指令与反馈监督", size=22)

    # Cylinder: body sides, lower elliptical arc, and full top ellipse.
    x, y, w, h, eh = 850, 745, 260, 105, 26
    f.d.rectangle((x, y+eh/2, x+w, y+h-eh/2), fill="white")
    f.d.line([(x, y+eh/2), (x, y+h-eh/2)], fill="#111111", width=2)
    f.d.line([(x+w, y+eh/2), (x+w, y+h-eh/2)], fill="#111111", width=2)
    f.d.arc((x, y+h-eh, x+w, y+h), 0, 180, fill="#111111", width=2)
    f.d.ellipse((x, y, x+w, y+eh), fill="white", outline="#111111", width=2)
    f.svg.append(f'<path d="M{x},{y+eh/2} V{y+h-eh/2} A{w/2},{eh/2} 0 0 0 {x+w},{y+h-eh/2} V{y+eh/2}" fill="white" stroke="#111111" stroke-width="2"/>')
    f.svg.append(f'<ellipse cx="{x+w/2}" cy="{y+eh/2}" rx="{w/2}" ry="{eh/2}" fill="white" stroke="#111111" stroke-width="2"/>')
    f.text(980, 790, "DataHub", 25, "#111111", True)
    f.text(980, 823, "运行状态快照", 22, "#111111")

    panel(1200, 245, 410, 505, "调度与安全层")
    item(1235, 315, 340, 90, "主控制任务", ["ControlTask · 20 ms"])
    item(1235, 465, 340, 90, "调试 / 测距 / 维护调度", ["FreeRTOS 独立任务"], 23)
    item(1235, 615, 340, 90, "安全保护与故障恢复", ["急停 / 失联 / 堵转 / 离线"])

    panel(1665, 260, 350, 475, "输入层")
    item(1695, 335, 290, 85, "遥控器输入", ["ELRS / CRSF"])
    item(1695, 475, 290, 85, "视觉目标输入", ["电脑识别 → HC13"])
    item(1695, 615, 290, 85, "调试请求输入", ["SWD / RTT"])

    f.text(1020, 938, "功能任务层为 ControlTask 顺序调用的步骤；视觉识别在电脑端运行，DataHub 用于状态观测。", 23, "#333333")
    f.save("control_system_five_columns_20261114")


if __name__ == "__main__":
    overall()
    software()
    five_columns()
