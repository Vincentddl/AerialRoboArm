$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
$gcc = if ($compiler) { $compiler.Source } else { 'D:\app\CLion2025\CLion 2025.2.4\bin\mingw\bin\gcc.exe' }
$out = Join-Path $root 'outputs\startup_angle_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$savedPath = $env:PATH
$env:PATH = "$(Split-Path -Parent $gcc);$env:PATH"
try {
    $exe = Join-Path $out 'test_startup_angle.exe'
    & $gcc '-std=c11' '-Wall' '-Wextra' '-Werror' '-static' "-I$root\User\global\Inc" `
        "-I$root\User\drv\Inc" "-I$root\User\mod\Inc" "$root\tests\test_startup_angle.c" `
        "$root\User\mod\Src\mod_startup_angle.c" '-lm' '-o' $exe
    if ($LASTEXITCODE -ne 0) { throw 'Startup test compilation failed.' }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw 'Startup angle tests failed.' }
    $homeExe = Join-Path $out 'test_home_zero.exe'
    & $gcc '-std=c11' '-Wall' '-Wextra' '-Werror' '-static' "-I$root\User\global\Inc" `
        "-I$root\User\drv\Inc" "-I$root\User\mod\Inc" "$root\tests\test_home_zero.c" `
        "$root\User\mod\Src\mod_home_zero.c" '-lm' '-o' $homeExe
    if ($LASTEXITCODE -ne 0) { throw 'Home-zero test compilation failed.' }
    & $homeExe
    if ($LASTEXITCODE -ne 0) { throw 'Home-zero tests failed.' }
} finally { $env:PATH = $savedPath }
