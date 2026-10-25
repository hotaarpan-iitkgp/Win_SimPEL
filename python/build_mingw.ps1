# Builds the simpel wheel with the MinGW-w64 g++ toolchain and Ninja, then installs it
# into the Python that runs pip (use a venv). Run from anywhere:
#   powershell -ExecutionPolicy Bypass -File python\build_mingw.ps1 [-Python path\to\python.exe]
param([string]$Python = "python")
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$env:CMAKE_GENERATOR = 'Ninja'
$env:CXX = 'g++'
$env:CC = 'gcc'
& $Python -m pip install -r (Join-Path $here 'requirements-dev.txt')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $Python -m pip install --no-build-isolation --no-deps --force-reinstall $here
exit $LASTEXITCODE
