$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
$gcc = if ($compiler) { $compiler.Source } else { 'D:\app\CLion2025\CLion 2025.2.4\bin\mingw\bin\gcc.exe' }
$out = Join-Path $root 'outputs\jy60_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$savedPath = $env:PATH
$env:PATH = "$(Split-Path -Parent $gcc);$env:PATH"
try {
    & $gcc '-std=c11' '-Wall' '-Wextra' '-Werror' '-static' "-I$root\User\drv\Inc" `
        "$root\tests\test_jy60.c" "$root\User\drv\Src\drv_jy60.c" '-o' "$out\test_jy60.exe"
    if ($LASTEXITCODE -ne 0) { throw 'JY60 test compilation failed.' }
    & "$out\test_jy60.exe"
    if ($LASTEXITCODE -ne 0) { throw 'JY60 tests failed.' }
} finally { $env:PATH = $savedPath }
