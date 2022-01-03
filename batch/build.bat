@ECHO OFF
ECHO ============================================================
ECHO Building Cockroach .exe in Release Mode
ECHO ***be sure that MSBuild is in your eenvironment variable path***
ECHO ============================================================
MSBuild.exe -t:rebuild build/Cockroach.sln /property:Configuration=Release