@echo off
echo ========================================================
echo Building High Performance C++ Windows Desktop Application
echo ========================================================

if not exist build mkdir build
cd build

cmake -G "MinGW Makefiles" ..
if %ERRORLEVEL% NEQ 0 (
    echo CMake configuration failed!
    exit /b %ERRORLEVEL%
)

mingw32-make -j4
if %ERRORLEVEL% NEQ 0 (
    echo Compilation Failed!
    exit /b %ERRORLEVEL%
)

echo ========================================================
echo SUCCESS: circuitsim_pro_win.exe compiled successfully!
echo Location: %cd%\circuitsim_pro_win.exe
echo ========================================================
cd ..
