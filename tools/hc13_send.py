"""Send one HC-13 vision command and wait for the MCU acknowledgement."""

from __future__ import annotations

import argparse
import sys
import time

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send V,<angle>,<speed>,<confidence> and show MCU ACK replies."
    )
    parser.add_argument("--port", default="COM5", help="PC-side HC-13 serial port")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--angle", type=int, required=True, help="servo g angle, -100..100 deg")
    parser.add_argument("--speed", type=int, default=0, help="0 uses the MCU AUTO default")
    parser.add_argument("--confidence", type=int, default=90, help="0..100")
    parser.add_argument("--repeat", type=int, default=10, help="number of packets to send")
    parser.add_argument("--interval", type=float, default=0.10, help="seconds between packets")
    parser.add_argument("--timeout", type=float, default=2.0, help="final ACK wait time")
    parser.add_argument(
        "--send-only",
        action="store_true",
        help="one-way test: do not require an MCU ACK",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not -100 <= args.angle <= 100:
        raise SystemExit("ERROR: --angle must stay within the MCU safety limit -100..100")
    if not 0 <= args.speed <= 65535:
        raise SystemExit("ERROR: --speed must be 0..65535")
    if not 0 <= args.confidence <= 100:
        raise SystemExit("ERROR: --confidence must be 0..100")
    if args.repeat < 1 or args.interval < 0 or args.timeout < 0:
        raise SystemExit("ERROR: repeat/interval/timeout values are invalid")

    packet = f"V,{args.angle},{args.speed},{args.confidence}\r\n".encode("ascii")
    received_ack = False
    line_buffer = bytearray()

    print(f"OPEN {args.port} @ {args.baud} 8N1")
    # The HC-USB-T/CH340 fixture can route DTR/RTS to the HC-13 red
    # configuration key.  PySerial defaults these lines active, which makes
    # ordinary payloads return ERROR as if they were invalid AT commands.
    # Set both inactive before opening so the module stays in transparent mode.
    link = serial.Serial(
        port=None,
        baudrate=args.baud,
        timeout=0,
        write_timeout=0.2,
        rtscts=False,
        dsrdtr=False,
    )
    link.dtr = False
    link.rts = False
    link.port = args.port
    link.open()
    with link:
        time.sleep(0.10)
        link.reset_input_buffer()
        for index in range(args.repeat):
            link.write(packet)
            link.flush()
            print(f"TX {index + 1:02d}/{args.repeat}: {packet.decode('ascii').strip()}")
            deadline = time.monotonic() + args.interval
            while time.monotonic() < deadline:
                received_ack |= read_replies(link, line_buffer)
                time.sleep(0.005)

        if args.send_only:
            print(f"DONE: sent {len(packet) * args.repeat} bytes; check MCU hc13/vf counters.")
            return 0

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            received_ack |= read_replies(link, line_buffer)
            if received_ack:
                # Continue briefly so a complete final reply can arrive.
                time.sleep(0.05)
                received_ack |= read_replies(link, line_buffer)
                break
            time.sleep(0.005)

    if received_ack:
        print("PASS: MCU received and parsed the HC-13 command.")
        return 0
    print("FAIL: no MCU ACK received. Check both HC-13 modules, wiring and baud rate.")
    return 2


def read_replies(link: serial.Serial, line_buffer: bytearray) -> bool:
    waiting = link.in_waiting
    if waiting:
        line_buffer.extend(link.read(waiting))

    ack = False
    while b"\n" in line_buffer:
        raw, _, remainder = line_buffer.partition(b"\n")
        line_buffer[:] = remainder
        text = raw.rstrip(b"\r").decode("ascii", errors="replace")
        if text:
            print(f"RX: {text}")
        fields = text.split(",")
        if len(fields) == 4 and fields[0] == "ACK":
            ack = True
    return ack


if __name__ == "__main__":
    sys.exit(main())
