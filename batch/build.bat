@ECHO OFF
ECHO ============================================================
ECHO Building the project linked to Cockroach in Release and run the .exe
ECHO ============================================================
rmdir build
mkdir build
for %%I in (.) do set CurrDirName=%%~nxI
ECHO %CurrDirName%
MSBuild.exe -t:rebuild build/%CurrDirName%.sln /property:Configuration=Release
"build/Release/%CurrDirName%.exe"