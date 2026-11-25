$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
$gcc = if ($compiler) { $compiler.Source } else { 'D:\app\CLion2025\CLion 2025.2.4\bin\mingw\bin\gcc.exe' }
$out = Join-Path $root 'outputs\hc13_host_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$savedPath = $env:PATH
$env:PATH = "$(Split-Path -Parent $gcc);$env:PATH"
try {
    & $gcc '-std=c11' '-Wall' '-Wextra' '-Werror' '-static' "-I$root\User\mod\Inc" "-I$root\User\app\Inc" `
        "$root\tests\test_hc13_host.c" "$root\User\mod\Src\mod_hc13_host.c" '-o' "$out\test_hc13_host.exe"
    if ($LASTEXITCODE -ne 0) { throw 'HC13 host tests compilation failed.' }
    & "$out\test_hc13_host.exe"
    if ($LASTEXITCODE -ne 0) { throw 'HC13 host tests failed.' }
} finally { $env:PATH = $savedPath }
