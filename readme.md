# **TEMP**

## For Windows

## Library installation

To build the library download CMake and from the terminal go to the root directory of Cockroach

```terminal
cmake -S . -B build -G "Visual Studio 16 2019" -A x64
```

Next, since it is a header-only librarythere is no need to build it and we just need to install it via cmake:

```terminal
cmake --build build --config Release --target install
```

By default the library will go in `C:/Program Files`.


## Usage

See the CMakeList.txt file in the `examples/cli`.
Go to this directory and type into the terminal the following:


```terminal
cmake -S . -B build -G "Visual Studio 16 2019" -A x64
```

```terminal
cmake --build build --config Release
```

or alternatively by using the MSbuild.exe. Just be sure that it is in your environment path.

```terminal
MSBuild.exe -t:rebuild build/Cockroach.sln /property:Configuration=Release
```

Cockroach headers should be referenced and the .exe should work.