@ECHO OFF
ECHO ============================================================
ECHO Building the project in Release mode
ECHO ============================================================
ECHO Be sure to run this command from the source directory...

:: For building with MSBuild
@REM for %%I in (.) do set CurrDirName=%%~nxI
@REM ECHO %CurrDirName%
@REM MSBuild.exe -t:rebuild build/%CurrDirName%.sln /property:Configuration=Release

cmake --build build --config Release