$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
$bundledGcc = 'E:\software\CLion 2025.3\bin\mingw\bin\gcc.exe'
$clionGcc = 'C:\CLionToolchains\mingw64\bin\gcc.exe'
$gcc = if ($compiler) { $compiler.Source } elseif (Test-Path -LiteralPath $clionGcc) { $clionGcc } elseif (Test-Path -LiteralPath $bundledGcc) { $bundledGcc } else {
    throw 'gcc.exe not found in PATH or the configured CLion installation.'
}
# MinGW ld.exe cannot reliably create an output beneath the Chinese workspace
# path on Windows. Keep the test executable in a short ASCII temp path.
$out = Join-Path $env:TEMP 'aerial_roboarm_hc13_host_tests'
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
