$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root 'cmake-build-hc13-host'
& cmake -S $root -B $build -G Ninja -DCMAKE_BUILD_TYPE=Debug `
    -DHX8_JY60_CAPTURE=ON -DJY61P_CAPTURE=ON -DHC13_HOST_LINK=ON `
    -DJY61P_DIAGNOSTIC=OFF -DJY60_UART_DEMO=OFF -DSERVO1_ONLY_TEST=OFF
if ($LASTEXITCODE -ne 0) { throw 'HC13 host configuration failed.' }
& cmake --build $build --parallel
if ($LASTEXITCODE -ne 0) { throw 'HC13 host build failed.' }
Write-Host "Built: $build\FOC_DEMO2.elf"
Write-Host 'Build only. No flashing and no actuator commands were performed.'
