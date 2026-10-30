# Documentation Index

This directory contains the curated documentation for the AerialRoboArm electrical-control subsystem. The repository does not include the undergraduate thesis document itself; these files provide the engineering context, contribution boundary, runtime architecture, hardware notes, demo notes, and historical archive.

## Primary Documents

| Document | Purpose |
| --- | --- |
| [MY_WORK.md](./MY_WORK.md) | Personal contribution boundary and work summary. |
| [design/06_CONTROL_SYSTEM_REPLAN.md](./design/06_CONTROL_SYSTEM_REPLAN.md) | Control-system replan: RC/vision inputs, HX8/FSUS main-arm control, PWM gripper, task timing, arbitration, and safety. |
| [design/07_MONOCULAR_VISION_GRASP_PLAN.md](./design/07_MONOCULAR_VISION_GRASP_PLAN.md) | Detailed monocular vision and three-frame grasp plan: hardware, formulas, measured PC timing, conditional latency budget, comparisons, and validation. |
| [About AI Assist Pipeline - Share.md](./About%20AI%20Assist%20Pipeline%20-%20Share.md) | DACMAS AI-native development workflow and methodology. |
| [hardware/pinmap.md](./hardware/pinmap.md) | STM32 pin assignment and external wiring reference. |
| [hardware/JY60_IMU.md](./hardware/JY60_IMU.md) | JY60 supply, UART protocol, measurement limits and source discrepancies; not used for arm control yet. |
| [hardware/JY61P_CAPTURE.md](./hardware/JY61P_CAPTURE.md) | JY61P 200 Hz on former ELRS USART1: startup configuration, rate checks, wiring and capture. |
| [hardware/JY60_UART_DEMO.md](./hardware/JY60_UART_DEMO.md) | Sensor-only JY60 demo using the former ELRS USART1 connector, flash/RTT instructions and hardware verification. |
| [hardware/HX8_JY60_CAPTURE.md](./hardware/HX8_JY60_CAPTURE.md) | Synchronized HX8 command and JY60 recording: VOFA 9090, capture 9091, MCU timestamps, automatic start/end. |
| [hardware/HC13_HOST_PROTOCOL_V1.md](./hardware/HC13_HOST_PROTOCOL_V1.md) | Host interface contract: sensor telemetry, HX8 and gripper commands, modes, acknowledgments and completion. Specification only; firmware/GUI integration pending. |
| [hardware/vl53l1x_grasp_gate.md](./hardware/vl53l1x_grasp_gate.md) | VL53L1X wiring and gripper gate notes for the active I2C distance-sensor path. |
| [hardware/vl53l0x_tof.md](./hardware/vl53l0x_tof.md) | Superseded GY-VL53L0X bring-up notes retained for rollback. |
| [demo/final-demo.md](./demo/final-demo.md) | Final `demo_v6` demo/testbench scope and media entry points. |
| [design/02_BUS_SERVO_CONTROL_MODEL.md](./design/02_BUS_SERVO_CONTROL_MODEL.md) | Control-theory modeling process for the HX8/FSUS bus-servo path. |
| [design/03_VISION_PERCEPTION_MODULE.md](./design/03_VISION_PERCEPTION_MODULE.md) | Vision perception module plan for lens calibration, target recognition, and base-relative offset estimation. |
| [design/04_Z_AXIS_TRAJECTORY_FEASIBILITY_PLAN.md](./design/04_Z_AXIS_TRAJECTORY_FEASIBILITY_PLAN.md) | Feasibility plan, validation gates, and rollout criteria for Z-axis-constrained target prediction and grasping. |

## Archived / Historical Documents

| Document | Archive reason |
| --- | --- |
| [design/00_SYSTEM_CONSTITUTION.md](./design/00_SYSTEM_CONSTITUTION.md) | Older BLDC/AS5600 design narrative; retained for thesis/history context. |
| [design/01_RUNTIME_ARCHITECTURE.md](./design/01_RUNTIME_ARCHITECTURE.md) | Older dual-thread BLDC/AS5600 runtime architecture; current bring-up uses HX8/FSUS + PTK. |
| [hardware/cubemx-demo-v7-checklist.md](./hardware/cubemx-demo-v7-checklist.md) | Old ST3215 half-duplex CubeMX checklist; current chain is STM32 USART2 -> UC-01 -> HX8-U26H-M. |

## Directory Layout

```text
Doc/
  README.md              Documentation index
  MY_WORK.md             Personal contribution summary
  design/                Formal architecture and design documents
  hardware/              Pin map and hardware-context assets
  demo/                  Final demo/testbench notes
  archive/               Historical notes, drafts, legacy assets, and mirrored vendor files
```

## Archive Policy

`archive/` keeps development-history materials that are useful for traceability but should not be treated as the current project entrance. This includes meeting notes, branch retrospectives, early protocol drafts, legacy figures, and the duplicated `Doc/Drivers` mirror that previously lived directly under `Doc/`.
