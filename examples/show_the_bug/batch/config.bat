@ECHO OFF
ECHO ============================================================
ECHO Configuring Project
ECHO ============================================================
rmdir build
mkdir build
cmake -S . -B build -G "Visual Studio 16 2019" -A x64