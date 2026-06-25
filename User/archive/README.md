# User Archive

This directory stores source files from older hardware or demo paths that are
kept for reference but are no longer part of the active firmware build.

Current CMake rules exclude `User/archive/**` from the firmware target. If a
file here is needed again, move it back into the active `User/` layer and update
the current hardware documentation first.

## Contents

| Path | Status |
| --- | --- |
| `drivers/st3215/` | Legacy ST3215 protocol driver. Current HX8-U26H-M path uses `drv_fsus`. |
