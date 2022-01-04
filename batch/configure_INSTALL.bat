@ECHO OFF
ECHO ============================================================
ECHO Building Cockroach sln
ECHO ============================================================
rmdir build
mkdir build
 cmake --build build --config Release --target install