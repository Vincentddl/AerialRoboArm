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
& $gcc -std=c11 -Wall -Wextra -Werror -static `
    "-I$root\User\mod\Inc" "-I$root\User\drv\Inc" "-I$root\User\global\Inc" `
    "$root\tests\test_position_trim.c" "$root\User\mod\Src\mod_position_trim.c" `
    "$root\User\drv\Src\drv_fsus.c" -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Controller test compilation failed.' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Controller tests failed.' }
} finally {
    $env:PATH = $savedPath
}
