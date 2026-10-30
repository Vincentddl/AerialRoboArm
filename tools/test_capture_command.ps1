$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
$gcc = if ($compiler) { $compiler.Source } else { 'D:\app\CLion2025\CLion 2025.2.4\bin\mingw\bin\gcc.exe' }
$out = Join-Path $root 'outputs\capture_command_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$savedPath = $env:PATH
$env:PATH = "$(Split-Path -Parent $gcc);$env:PATH"
try {
    & $gcc '-std=c11' '-Wall' '-Wextra' '-Werror' '-static' "-I$root\User\drv\Inc" `
        "$root\tests\test_capture_command.c" "$root\User\drv\Src\drv_capture_command.c" '-o' "$out\test_capture_command.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Capture command test compilation failed.' }
    & "$out\test_capture_command.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Capture command tests failed.' }
} finally { $env:PATH = $savedPath }
