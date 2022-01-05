@ECHO OFF
ECHO ============================================================
ECHO Building Cockroach sln
ECHO ============================================================
rmdir build
mkdir build
cmake -S . -B build -G "Visual Studio 16 2019" -A x64 -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_MODULE=OFF -DBUILD_CACHED_CUDA_MANAGER=OFF