# HC13 Standalone Configuration

This project keeps the main STM32 firmware separate from first-time HC13 setup.
Use `tools/hc13_config.ps1` with a USB-TTL adapter to configure each HC13 module
before connecting the receiver module to the MCU.

## Recommended Parameters

| Item | Value | Note |
| --- | --- | --- |
| Wireless mode | `S7` | Highest normal-mode throughput; manual lists about `19.0 KBytes/s`. |
| UART baud | `230400` | Higher than S7 throughput, so the UART is not the bottleneck. |
| Channel | `C043` | Default channel; both modules must match. |
| UART format | `8N1` | Project default and HC13 factory default format. |

## Wiring For Configuration

Connect one HC13 to a USB-TTL adapter:

| USB-TTL | HC13 |
| --- | --- |
| `3V3` | `VCC` |
| `GND` | `GND` |
| `TXD` | `RXD` |
| `RXD` | `TXD` |
| `GND` | `KEY` while configuring |

Keep `KEY` low before running the script. If `KEY` is pulled low before module
power-on, the manual says AT mode uses `9600,N,1`, which is the safest entry.

## Configure One Module

From the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File tools\hc13_config.ps1 -Port COM8
```

Repeat for the second module. The script sends:

```text
AT
AT+S7
AT+C043
AT+B230400
```

After it succeeds, release `KEY` high or disconnect it from `GND`, then wait at
least `30 ms` before using transparent mode.

## Query Only

```powershell
powershell -ExecutionPolicy Bypass -File tools\hc13_config.ps1 -Port COM8 -QueryOnly
```

The script probes `9600`, `115200`, and `230400` baud by default. This helps if
the module has already been configured before.

## Important Notes

- Configure both HC13 modules with the same wireless mode and channel.
- After setting `AT+B230400`, the PC serial tool and STM32 `USART3` must also use
  `230400`.
- HC13 stores AT parameters across power cycles, so first-time setup does not
  need to be repeated every boot.
- Keep packets under `1000 bytes`; HC13 is half-duplex and cannot send and
  receive over the air at the same time.
