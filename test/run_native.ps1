param([string]$Compiler = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
Push-Location $projectRoot
try {
  if (-not $Compiler) {
    $localCompiler = Join-Path $projectRoot '.pio/host-tools/w64devkit/bin/g++.exe'
    if (Test-Path -LiteralPath $localCompiler) { $Compiler = $localCompiler }
    else { $Compiler = (Get-Command g++ -ErrorAction Stop).Source }
  }
  New-Item -ItemType Directory -Force '.pio/native-tests' | Out-Null
  foreach ($suite in Get-ChildItem test -Directory -Filter 'test_*') {
    $testSource = Join-Path $suite.FullName 'test_main.cpp'
    if (-not (Test-Path -LiteralPath $testSource)) { continue }
    $binary = Join-Path $projectRoot ('.pio/native-tests/' + $suite.Name + '.exe')
    $flags = @()
    if ($suite.Name -in @('test_serial_diagnostics', 'test_calibration_safety')) {
      $flags = @('-DARDUINO', '-Itest/support')
    }
    & $Compiler -std=c++17 -Wall -Wextra -Werror -Iinclude @flags $testSource src/Joint.cpp src/Leg.cpp src/Kinematics.cpp src/Motion.cpp -o $binary
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($suite.Name)" }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $($suite.Name)" }
  }
} finally { Pop-Location }
