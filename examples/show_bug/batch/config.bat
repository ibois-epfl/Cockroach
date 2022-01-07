@ECHO OFF
ECHO ============================================================
ECHO Configuring Project
ECHO ============================================================

:: Check for build/ delete or create
if exist build\ (
    ECHO build\ exists and it is being erased
    rmdir /Q /S build
    mkdir build
) else (
    ECHO build\ does not exist and it is being created
    mkdir build
)

:: Configure the project
ECHO Configuring the CMake project now...
cmake -S . -B build -G "Visual Studio 16 2019" -A x64 -DCilantro_DIR="C:\Program Files\cilantro\lib\cmake\cilantro" -DCilantro_INCLUDE_DIR="C:\Program Files\cilantro\include"