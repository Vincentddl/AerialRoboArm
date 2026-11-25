"""同一个串口实例上的异步收发。串口对象只在工作线程内访问。"""
from __future__ import annotations

import queue
import threading
import time

import serial
from serial.tools import list_ports
from PySide6.QtCore import QObject, QThread, Signal


def available_ports() -> list[tuple[str, str]]:
    return [(p.device, p.description or p.device) for p in list_ports.comports()]


class SerialWorker(QThread):
    opened = Signal()
    received = Signal(bytes, float)
    failed = Signal(str)
    write_result = Signal(bytes, bool, str)
    wire_written = Signal(bytes, bool, str, float)

    def __init__(self, port: str, baud: int, parent: QObject | None = None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._stop = threading.Event()
        self._outgoing: queue.Queue[bytes] = queue.Queue(maxsize=32)
        self._urgent: queue.Queue[bytes] = queue.Queue(maxsize=4)

    def send(self, payload: bytes, *, priority: bool = False) -> bool:
        if not payload or self._stop.is_set():
            return False
        try:
            (self._urgent if priority else self._outgoing).put_nowait(payload)
            return True
        except queue.Full:
            return False

    def stop(self) -> None:
        self._stop.set()

    def discard_normal_queue(self) -> None:
        while True:
            try:
                self._outgoing.get_nowait()
            except queue.Empty:
                return

    def run(self) -> None:
        try:
            # pyserial 默认把 DTR/RTS 置高；无流控并不等于这两条线关闭。
            # 先在关闭状态设定电平，再打开端口，匹配 XCOM 的未勾选配置。
            link = serial.Serial(
                port=None, baudrate=self.baud, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0.1, write_timeout=0.3, xonxoff=False, rtscts=False,
                dsrdtr=False,
            )
            link.dtr = False
            link.rts = False
            link.port = self.port
            link.open()
            with link:
                if self._stop.is_set():
                    return
                self.opened.emit()
                while not self._stop.is_set():
                    try:
                        payload = self._urgent.get_nowait()
                    except queue.Empty:
                        try:
                            payload = self._outgoing.get_nowait()
                        except queue.Empty:
                            payload = None
                    if payload is not None:
                        try:
                            written = link.write(payload)
                            if written != len(payload):
                                raise serial.SerialTimeoutException("写入字节数不足")
                            detail = f"已写入 {written} 字节，等待设备ACK"
                            self.wire_written.emit(payload, True, detail, time.monotonic())
                            self.write_result.emit(payload, True, detail)
                        except (serial.SerialException, OSError) as exc:
                            detail = f"串口写入失败：{exc}"
                            self.wire_written.emit(payload, False, detail, time.monotonic())
                            self.write_result.emit(payload, False, detail)
                            raise
                    # CH340 在连续低速数据流中可能等满请求长度才返回。
                    # 空闲时只等首字节，已有数据则读取当前缓存，避免 ACK 延迟数秒。
                    chunk = link.read(min(max(link.in_waiting, 1), 4096))
                    if chunk:
                        self.received.emit(chunk, time.monotonic())
        except (serial.SerialException, OSError, ValueError) as exc:
            if not self._stop.is_set():
                self.failed.emit(f"串口打开或通信失败：{exc}")


class SerialService(QObject):
    state_changed = Signal(object)
    raw_received = Signal(bytes, float)
    error = Signal(str)
    write_result = Signal(bytes, bool, str)
    wire_written = Signal(bytes, bool, str, float)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        from .models import ConnectionState
        self.state = ConnectionState.DISCONNECTED
        self.worker: SerialWorker | None = None
        self._closing = False

    def _set_state(self, state) -> None:
        self.state = state
        self.state_changed.emit(state)

    def connect_port(self, port: str, baud: int) -> None:
        from .models import ConnectionState
        if self.worker is not None or not port:
            return
        self._closing = False
        self._set_state(ConnectionState.CONNECTING)
        worker = SerialWorker(port, baud, self)
        self.worker = worker
        worker.opened.connect(lambda: self._set_state(ConnectionState.OPEN) if not self._closing else None)
        worker.received.connect(self.raw_received)
        worker.failed.connect(self._failed)
        worker.write_result.connect(self.write_result)
        worker.wire_written.connect(self.wire_written)
        worker.finished.connect(lambda: self._finished(worker))
        worker.start()

    def _failed(self, message: str) -> None:
        from .models import ConnectionState
        self._set_state(ConnectionState.FAILED)
        self.error.emit(message)

    def _finished(self, worker: SerialWorker) -> None:
        from .models import ConnectionState
        if self.worker is worker:
            self.worker = None
        if self.state != ConnectionState.FAILED:
            self._set_state(ConnectionState.DISCONNECTED)
        worker.deleteLater()

    def disconnect_port(self) -> None:
        from .models import ConnectionState
        if self.worker is None:
            self._set_state(ConnectionState.DISCONNECTED)
            return
        self._closing = True
        self._set_state(ConnectionState.DISCONNECTING)
        self.worker.stop()

    def send(self, payload: bytes, *, priority: bool = False) -> bool:
        from .models import ConnectionState
        return self.state == ConnectionState.OPEN and self.worker is not None and self.worker.send(payload, priority=priority)

    def discard_normal_queue(self) -> None:
        if self.worker is not None:
            self.worker.discard_normal_queue()

    def shutdown(self) -> None:
        self._closing = True
        worker = self.worker
        if worker is not None:
            worker.stop()
            worker.wait(1800)
