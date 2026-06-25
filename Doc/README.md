# Documentation Index

This directory contains the curated documentation for the AerialRoboArm electrical-control subsystem. The repository does not include the undergraduate thesis document itself; these files provide the engineering context, contribution boundary, runtime architecture, hardware notes, demo notes, and historical archive.

## Primary Documents

| Document | Purpose |
| --- | --- |
| [MY_WORK.md](./MY_WORK.md) | Personal contribution boundary and work summary. |
| [About AI Assist Pipeline - Share.md](./About%20AI%20Assist%20Pipeline%20-%20Share.md) | DACMAS AI-native development workflow and methodology. |
| [hardware/pinmap.md](./hardware/pinmap.md) | STM32 pin assignment and external wiring reference. |
| [demo/final-demo.md](./demo/final-demo.md) | Final `demo_v6` demo/testbench scope and media entry points. |

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
