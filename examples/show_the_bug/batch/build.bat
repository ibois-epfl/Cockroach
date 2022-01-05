@ECHO OFF
ECHO ============================================================
ECHO Building the project
ECHO ============================================================
rmdir build
mkdir build
for %%I in (.) do set CurrDirName=%%~nxI
ECHO %CurrDirName%
MSBuild.exe -t:rebuild build/%CurrDirName%.sln /property:Configuration=Release
"build/Release/%CurrDirName%.exe"