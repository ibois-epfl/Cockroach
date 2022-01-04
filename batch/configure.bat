@ECHO OFF
ECHO ============================================================
ECHO Building Cockroach sln
ECHO ============================================================
rmdir build
mkdir build
cmake -S . -B build/ -DBUILD_TESTING=OFF