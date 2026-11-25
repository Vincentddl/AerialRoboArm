"""限时 HC13 V2 只读排障：被动监听，然后仅查询 INFO/CAL/STATE/PING。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import secrets
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if (ROOT / "vendor").exists():
    sys.path.insert(0, str(ROOT / "vendor"))
sys.path.insert(0, str(ROOT))

import serial
from serial.tools import list_ports

from arm_console.v1_protocol import V1Decoder, V1Encoder


def main() -> int:
    parser = argparse.ArgumentParser(description="HC13 V2 只读串口探测；绝不发送运动命令")
    parser.add_argument("--port", required=True, help="例如 COM4")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--passive-seconds", type=float, default=2.0)
    parser.add_argument("--reply-seconds", type=float, default=0.6)
    args = parser.parse_args()
    if not 0 <= args.passive_seconds <= 30 or not 0.1 <= args.reply_seconds <= 5:
        parser.error("探测时间超出限制")

    out_dir = ROOT / "diagnostics"
    out_dir.mkdir(exist_ok=True)
    path = out_dir / f"hc13_probe_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    decoder = V1Decoder()
    received = 0
    valid = []

    with path.open("w", encoding="utf-8") as log:
        def record(kind: str, **fields: object) -> None:
            item = {"wall_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                    "monotonic_s": time.monotonic(), "kind": kind, **fields}
            log.write(json.dumps(item, ensure_ascii=False) + "\n")
            log.flush()

        ports = [{"port": p.device, "description": p.description, "hwid": p.hwid}
                 for p in list_ports.comports()]
        record("start", port=args.port, baud=args.baud, config="8N1/no-flow/DTR=off/RTS=off",
               safe_commands=["GET INFO", "GET CAL", "GET STATE", "PING"], ports=ports)

        def collect(link: serial.Serial, seconds: float) -> None:
            nonlocal received
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                chunk = link.read(min(max(link.in_waiting, 1), 4096))
                if chunk:
                    received += len(chunk)
                    frames = decoder.feed(chunk, time.monotonic())
                    names = [type(frame).__name__ for frame in frames]
                    valid.extend(name for name in names if name != "InvalidFrame")
                    record("RX", hex=chunk.hex(" ").upper(), ascii=repr(chunk), parsed=names)

        try:
            link = serial.Serial(port=None, baudrate=args.baud, bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                 timeout=0.05, write_timeout=0.3, xonxoff=False,
                                 rtscts=False, dsrdtr=False)
            link.dtr = False
            link.rts = False
            link.port = args.port
            link.open()
            with link:
                record("opened")
                collect(link, args.passive_seconds)
                request_id = secrets.randbelow(1_000_000_000) + 1
                for verb, value in (("GET", "INFO"), ("GET", "CAL"),
                                    ("GET", "STATE"), ("PING", None)):
                    payload = (V1Encoder.encode(verb, request_id, value) if value
                               else V1Encoder.encode(verb, request_id))
                    try:
                        count = link.write(payload)
                        record("TX", req_id=request_id, command=f"{verb} {value or ''}".strip(),
                               hex=payload.hex(" ").upper(), ascii=repr(payload), written=count)
                        if count != len(payload):
                            record("error", detail="串口部分写入")
                            break
                    except (serial.SerialException, OSError) as exc:
                        record("error", detail=f"写入失败：{exc}")
                        break
                    collect(link, args.reply_seconds)
                    request_id += 1
                collect(link, 0.5)
                record("end", rx_bytes=received, parsed_frames=valid)
        except (serial.SerialException, OSError, ValueError) as exc:
            record("error", detail=f"打开或读取失败：{exc}")
            print(f"串口探测失败：{exc}")
            print(f"原始记录：{path}")
            return 2
    print(f"探测完成：接收 {received} 字节，有效协议帧 {len(valid)} 条")
    print(f"原始记录：{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
