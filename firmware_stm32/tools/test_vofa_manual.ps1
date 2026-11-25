$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
$gcc = if ($compiler) { $compiler.Source } else { 'D:\app\CLion2025\CLion 2025.2.4\bin\mingw\bin\gcc.exe' }
$out = Join-Path $root 'outputs\vofa_manual_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$exe = Join-Path $out 'test_vofa_manual.exe'
$savedPath = $env:PATH
$env:PATH = "$(Split-Path -Parent $gcc);$env:PATH"
try {
& $gcc '-std=c11' '-Wall' '-Wextra' '-Werror' '-static' `
    "-I$root\User\task\Inc" "-I$root\User\mod\Inc" "-I$root\User\drv\Inc" `
    "-I$root\User\bsp\Inc" "-I$root\User\global\Inc" `
    "$root\tests\test_vofa_manual.c" "$root\User\task\Src\task_arbiter.c" `
    "$root\User\mod\Src\mod_home_zero.c" "$root\User\mod\Src\mod_startup_angle.c" '-lm' '-o' $exe
if ($LASTEXITCODE -ne 0) { throw 'VOFA manual test compilation failed.' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'VOFA manual tests failed.' }
} finally { $env:PATH = $savedPath }
