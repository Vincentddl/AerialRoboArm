param([switch]$CompareBaseline)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
$gcc = if ($compiler) { $compiler.Source } else { 'D:\app\CLion2025\CLion 2025.2.4\bin\mingw\bin\gcc.exe' }
if (-not (Test-Path -LiteralPath $gcc)) { throw 'Native MinGW gcc is required to run the controller tests.' }
$out = Join-Path $root 'outputs\position_trim_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$exe = Join-Path $out 'test_position_trim.exe'
$savedPath = $env:PATH
$env:PATH = "$(Split-Path -Parent $gcc);$env:PATH"
try {
    $compile = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-static',
        "-I$root\User\mod\Inc", "-I$root\User\drv\Inc", "-I$root\User\global\Inc",
        "$root\tests\test_position_trim.c", "$root\User\mod\Src\mod_position_trim.c",
        "$root\User\drv\Src\drv_fsus.c", '-lm')
    & $gcc @compile -o $exe
    if ($LASTEXITCODE -ne 0) { throw 'Controller test compilation failed.' }
    $fastOutput = @(& $exe)
    if ($LASTEXITCODE -ne 0) { throw 'Controller tests failed.' }
    $fastOutput | Write-Output
    if ($CompareBaseline) {
        $baselineExe = Join-Path $out 'test_position_trim_baseline.exe'
        & $gcc @compile '-DPOSITION_TRIM_SETTLE_MS=600U' '-DPOSITION_TRIM_INTERVAL_MS=200U' `
            '-DPOSITION_TRIM_QUIET_SAMPLES=3U' '-DPOSITION_TRIM_MAX_STEP_DEG=0.10f' -o $baselineExe
        if ($LASTEXITCODE -ne 0) { throw 'Baseline test compilation failed.' }
        $baselineOutput = @(& $baselineExe)
        if ($LASTEXITCODE -ne 0) { throw 'Baseline controller tests failed.' }
        Write-Output 'Baseline (600 ms / 200 ms / 0.1 deg):'
        $baselineOutput | Write-Output
        $pattern = 'bias=\+1\.2:.*settled_ms=(\d+)'
        $fastMatch = [regex]::Match(($fastOutput -join "`n"), $pattern)
        $baselineMatch = [regex]::Match(($baselineOutput -join "`n"), $pattern)
        if (-not $fastMatch.Success -or -not $baselineMatch.Success) { throw 'Settling comparison output missing.' }
        $fastMs = [int]$fastMatch.Groups[1].Value
        $baselineMs = [int]$baselineMatch.Groups[1].Value
        if ($fastMs -gt $baselineMs) { throw 'Fast controller regressed settling time.' }
        @{fast_settled_ms=$fastMs; baseline_settled_ms=$baselineMs; simulation_only=$true} |
            ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out 'comparison.json') -Encoding UTF8
    }
} finally {
    $env:PATH = $savedPath
}
