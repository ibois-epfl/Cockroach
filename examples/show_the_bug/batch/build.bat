@ECHO OFF
ECHO ============================================================
ECHO Building the project in Release mode
ECHO ============================================================
ECHO Be sure to run this command from the source directory...

for %%I in (.) do set CurrDirName=%%~nxI
ECHO %CurrDirName%
MSBuild.exe -t:rebuild build/%CurrDirName%.sln /property:Configuration=Release
@REM "build/Release/%CurrDirName%.exe"