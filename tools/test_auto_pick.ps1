$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
$gcc = if ($compiler) { $compiler.Source } else { 'D:\app\CLion2025\CLion 2025.2.4\bin\mingw\bin\gcc.exe' }
if (-not (Test-Path -LiteralPath $gcc)) { throw 'Native MinGW gcc is required.' }
$out = Join-Path $root 'outputs\auto_pick_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$exe = Join-Path $out 'test_auto_pick.exe'
$savedPath = $env:PATH
$env:PATH = "$(Split-Path -Parent $gcc);$env:PATH"
try {
  foreach ($scheme in @(0, 1)) {
    & $gcc '-std=c11' '-Wall' '-Wextra' '-Werror' '-static' "-DTASK_MANIP_AUTO_GRIP_THREE_FRAME=$scheme" `
        "-I$root\User\task\Inc" "-I$root\User\mod\Inc" "-I$root\User\drv\Inc" `
        "-I$root\User\bsp\Inc" "-I$root\User\global\Inc" `
        "$root\tests\test_auto_pick.c" "$root\User\task\Src\task_manipulator.c" "$root\User\task\Src\task_arbiter.c" "$root\User\drv\Src\drv_hc13.c" '-lm' '-o' $exe
    if ($LASTEXITCODE -ne 0) { throw 'AUTO pick test compilation failed.' }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw 'AUTO pick tests failed.' }
  }
} finally { $env:PATH = $savedPath }
