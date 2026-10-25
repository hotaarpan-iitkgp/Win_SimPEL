# Builds and runs the standalone engine test suites and checks the fixed-step
# reference digests in tests/REFERENCE.txt.
# Usage (from the "c++ version" directory):  powershell -File tests\run_engine_suites.ps1
# Writes a summary to tests\engine_suites.log and exits non-zero on any failure.
$ErrorActionPreference = 'Continue'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$out = Join-Path $env:TEMP 'simpel_engine_suites'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$log = Join-Path $root 'tests\engine_suites.log'
'' | Out-File -Encoding ascii $log

$core = @('src\engine\CircuitSimulator.cpp', 'src\engine\ExpressionEvaluator.cpp', 'src\engine\CScriptEngine.cpp')
$suites = [ordered]@{
    'test_rerun' = @(); 'test_magnetic' = @(); 'test_power_semis' = @(); 'test_telemetry' = @();
    'test_lucache' = @(); 'test_adaptive' = @(); 'test_wiring' = @('src\engine\NetlistParser.cpp');
    'dump_buck' = @(); 'esr_scope' = @()
}
$failed = 0
foreach ($name in $suites.Keys) {
    $exe = Join-Path $out "$name.exe"
    $srcs = @("tests\$name.cpp") + $core + $suites[$name]
    $buildOut = & g++ -std=c++17 -O2 -I src -I 3rdparty -o $exe @srcs 2>&1
    if ($LASTEXITCODE -ne 0) {
        "BUILD FAIL $name" | Out-File -Append -Encoding ascii $log
        $buildOut | Out-File -Append -Encoding ascii $log
        $failed++
    }
}

function Write-Log($s) { $s | Out-File -Append -Encoding ascii $log }

foreach ($name in @('test_rerun', 'test_magnetic', 'test_power_semis', 'test_telemetry', 'test_lucache', 'test_adaptive', 'test_wiring')) {
    $exe = Join-Path $out "$name.exe"
    if (-not (Test-Path $exe)) { continue }
    $res = & $exe 2>&1
    $code = $LASTEXITCODE
    $tail = ($res | Select-Object -Last 1)
    Write-Log ("{0,-18} exit={1} {2}" -f $name, $code, $tail)
    if ($code -ne 0) { $failed++ }
}

# Reference digests.
$ref = Get-Content (Join-Path $root 'tests\REFERENCE.txt')
foreach ($circuit in @('buck', 'mathmix')) {
    foreach ($solver in @('euler', 'trapezoidal', 'rk4')) {
        $tmp = Join-Path $out "dump_${circuit}_$solver.txt"
        cmd /c "`"$(Join-Path $out 'dump_buck.exe')`" $solver 5e-3 $circuit > `"$tmp`""
        $h = (Get-FileHash -Algorithm SHA256 $tmp).Hash
        $want = ($ref | Where-Object { $_ -match "^dump $circuit\s+$solver\s+" }) -replace '^.*\s', ''
        $ok = ($h -eq $want)
        if (-not $ok) { $failed++ }
        Write-Log ("dump {0,-8} {1,-12} {2} {3}" -f $circuit, $solver, $(if ($ok) { 'OK' } else { 'MISMATCH' }), $h)
    }
}
foreach ($esr in @('0', '0.01')) {
    $lines = & (Join-Path $out 'esr_scope.exe') $esr 2>&1
    foreach ($l in $lines) {
        $ok = $ref -contains $l
        if (-not $ok) { $failed++ }
        Write-Log ("{0} {1}" -f $(if ($ok) { 'OK      ' } else { 'MISMATCH' }), $l)
    }
}
Write-Log "failed=$failed"
exit $failed
