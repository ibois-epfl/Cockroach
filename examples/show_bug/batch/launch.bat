@ECHO OFF
ECHO ============================================================
ECHO Launch .exe
ECHO ============================================================
ECHO Be sure to run this command from the source directory...

for %%I in (.) do set CurrDirName=%%~nxI
ECHO %CurrDirName%
START "RunningExe" /d "build/Release" %CurrDirName%.exe