# Batch files for windows terminal

Be sure to run the Windows terminal as an Administrator.

Then, just type the following to configure, build and launch the compiled .exe program (Release mode) from the main root of the project (i.e., C:\Program Files\Cockroach\build):

** to be confirmed if you build cmake with this one for header-only libraries **

```terminal
./batch/configure_INSTALL.bat
```



> 💬 Be sure to have the MSBuild.exe folder in the path environments.

```terminal
./batch/build.bat
```
```terminal
./batch/launch.bat
```


===========================
Second version

1/ Configuration in root folder of binaries
```terminal
mkdir build
cmake -S . -B build
```

2/ Installation
```terminal
cmake --build build --config Release --target install
```

Stop the header-only cockroach library is installed