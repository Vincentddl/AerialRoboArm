#!/usr/bin/env python3
"""Standalone UART monitor for the Nooploop TOFSense-F2 P."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable


FRAME_HEADER = b"\x57\x00"
FRAME_LENGTH = 16
DEFAULT_BAUD = 921_600


class FrameError(ValueError):
    """Raised when a TOFSense frame is incomplete or invalid."""


@dataclass(frozen=True)
class TOFSenseF2PFrame:
    sensor_id: int
    system_time_ms: int
    distance_m: float
    distance_status: int
    signal_strength: int
    range_precision_raw: int
    raw: bytes

    @property
    def distance_valid(self) -> bool:
        # Nooploop FAQ: TOFSense-F2 series uses 1=valid, 0=invalid.
        return self.distance_status == 1


def checksum(data: bytes) -> int:
    """Return the low eight bits of the byte sum."""
    return sum(data) & 0xFF


def decode_signed_le_24(data: bytes) -> int:
    """Decode a signed, little-endian 24-bit integer."""
    if len(data) != 3:
        raise ValueError("a 24-bit value must contain exactly three bytes")
    value = int.from_bytes(data, byteorder="little", signed=False)
    if value & 0x800000:
        value -= 1 << 24
    return value


def parse_frame(raw: bytes) -> TOFSenseF2PFrame:
    """Parse and validate one 16-byte TOFSense/F-series Frame0 packet."""
    if len(raw) != FRAME_LENGTH:
        raise FrameError(f"expected {FRAME_LENGTH} bytes, got {len(raw)}")
    if raw[:2] != FRAME_HEADER:
        raise FrameError(f"unexpected header: {raw[:2].hex(' ')}")
    expected = checksum(raw[:-1])
    if raw[-1] != expected:
        raise FrameError(
            f"checksum mismatch: received 0x{raw[-1]:02x}, expected 0x{expected:02x}"
        )

    return TOFSenseF2PFrame(
        sensor_id=raw[3],
        system_time_ms=int.from_bytes(raw[4:8], "little"),
        distance_m=decode_signed_le_24(raw[8:11]) / 1000.0,
        distance_status=raw[11],
        signal_strength=int.from_bytes(raw[12:14], "little"),
        range_precision_raw=raw[14],
        raw=bytes(raw),
    )


def build_query_frame(sensor_id: int) -> bytes:
    """Build the eight-byte Frame0 request used in UART query mode."""
    if not 0 <= sensor_id <= 0xFF:
        raise ValueError("sensor ID must be in the range 0..255")
    request = bytearray((0x57, 0x10, 0xFF, 0xFF, sensor_id, 0xFF, 0xFF, 0x00))
    request[-1] = checksum(request[:-1])
    return bytes(request)


def parse_sensor_id(value: str) -> int:
    """Parse a decimal/hex sensor ID for argparse."""
    try:
        sensor_id = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("模块 ID 必须是整数") from exc
    if not 0 <= sensor_id <= 0xFF:
        raise argparse.ArgumentTypeError("模块 ID 必须在 0..255 范围内")
    return sensor_id


class StreamDecoder:
    """Recover validated frames from an arbitrary UART byte stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> Iterable[TOFSenseF2PFrame]:
        self._buffer.extend(data)
        frames: list[TOFSenseF2PFrame] = []

        while True:
            start = self._buffer.find(FRAME_HEADER)
            if start < 0:
                # Keep a possible first header byte across reads.
                self._buffer[:] = self._buffer[-1:] if self._buffer[-1:] == b"\x57" else b""
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < FRAME_LENGTH:
                break

            candidate = bytes(self._buffer[:FRAME_LENGTH])
            try:
                frame = parse_frame(candidate)
            except FrameError:
                # Move one byte forward and search again. This also recovers
                # from a dropped UART byte or a false 0x57 0x00 sequence.
                del self._buffer[0]
                continue

            frames.append(frame)
            del self._buffer[:FRAME_LENGTH]

        return frames


def format_frame(frame: TOFSenseF2PFrame, include_raw: bool) -> str:
    validity = "VALID" if frame.distance_valid else "INVALID"
    line = (
        f"id={frame.sensor_id:3d}  sensor_time={frame.system_time_ms:10d} ms  "
        f"distance={frame.distance_m:8.3f} m  status={frame.distance_status} "
        f"({validity})  signal={frame.signal_strength:5d}  "
        f"range_precision={frame.range_precision_raw:3d}"
    )
    if include_raw:
        line += f"  raw={frame.raw.hex(' ')}"
    return line


def import_serial():
    try:
        import serial  # type: ignore[import-not-found]
        from serial.tools import list_ports  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit(
            "缺少 pyserial。请运行: python -m pip install -r requirements.txt"
        ) from exc
    return serial, list_ports


def list_serial_ports() -> int:
    _, list_ports = import_serial()
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口。")
        return 1
    for port in ports:
        print(f"{port.device:12s} {port.description}")
    return 0


def monitor(args: argparse.Namespace) -> int:
    serial, _ = import_serial()
    if not args.port:
        raise SystemExit("请通过 --port 指定串口，例如 --port COM6")

    decoder = StreamDecoder()
    received = 0
    next_query_at = 0.0
    query = build_query_frame(args.query_id) if args.query_id is not None else None

    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.read_timeout,
        ) as uart:
            uart.reset_input_buffer()
            mode = "query" if query is not None else "active"
            print(
                f"已打开 {uart.port} @ {uart.baudrate} bps, mode={mode}; "
                "按 Ctrl+C 停止。",
                file=sys.stderr,
            )

            while args.count == 0 or received < args.count:
                now = time.monotonic()
                if query is not None and now >= next_query_at:
                    uart.write(query)
                    uart.flush()
                    next_query_at = now + args.query_period

                chunk = uart.read(max(1, uart.in_waiting))
                for frame in decoder.feed(chunk):
                    print(format_frame(frame, args.raw), flush=True)
                    received += 1
                    if args.count and received >= args.count:
                        break
    except serial.SerialException as exc:
        print(f"串口错误: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n已停止。", file=sys.stderr)

    return 0


def self_test() -> int:
    sample = bytes.fromhex(
        "57 00 ff 00 9e 8f 00 00 ad 08 00 00 03 00 06 41"
    )
    frame = parse_frame(sample)
    assert frame.sensor_id == 0
    assert frame.system_time_ms == 36_766
    assert frame.distance_m == 2.221
    assert frame.distance_status == 0
    assert not frame.distance_valid
    assert frame.signal_strength == 3
    assert frame.range_precision_raw == 6
    assert build_query_frame(3) == bytes.fromhex("57 10 ff ff 03 ff ff 66")

    damaged = bytearray(sample)
    damaged[-1] ^= 0x01
    decoder = StreamDecoder()
    recovered: list[TOFSenseF2PFrame] = []
    recovered.extend(decoder.feed(b"noise\x57"))
    recovered.extend(decoder.feed(bytes(damaged) + sample[:7]))
    recovered.extend(decoder.feed(sample[7:]))
    assert recovered == [frame]

    print("SELF-TEST OK")
    print(format_frame(frame, include_raw=True))
    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="TOFSense-F2 P 独立 UART 测距读取程序",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", help="串口名，例如 Windows 的 COM6 或 Linux 的 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="UART 波特率")
    parser.add_argument("--count", type=int, default=0, help="读取指定帧数后退出；0 表示持续读取")
    parser.add_argument("--raw", action="store_true", help="同时显示原始 16 字节帧")
    parser.add_argument("--list-ports", action="store_true", help="列出串口后退出")
    parser.add_argument("--self-test", action="store_true", help="不连接硬件，执行协议解析自测")
    parser.add_argument(
        "--query-id",
        type=parse_sensor_id,
        help="启用查询模式，并指定模块 ID（支持十进制或 0x 前缀）",
    )
    parser.add_argument("--query-period", type=float, default=0.1, help="查询模式发送周期，单位秒")
    parser.add_argument("--read-timeout", type=float, default=0.1, help="串口单次读取超时，单位秒")
    return parser


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()
    if args.count < 0:
        parser.error("--count 不能为负数")
    if args.query_period <= 0:
        parser.error("--query-period 必须大于 0")
    if args.read_timeout <= 0:
        parser.error("--read-timeout 必须大于 0")
    if args.self_test:
        return self_test()
    if args.list_ports:
        return list_serial_ports()
    return monitor(args)


if __name__ == "__main__":
    raise SystemExit(main())
