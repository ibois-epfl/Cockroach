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
MSBuild.exe -t:rebuild build/<name-of-the-project>.sln /property:Configuration=Release
```

Cockroach headers should be referenced and the .exe should work.



==============================
12:01 05/01/2022

=================

**Install Eigen for Cilantro and Open3d**

-Clone the repo from here:
git clone https://gitlab.com/libeigen/eigen.git

=================

**Open3d**
***COMPILE LIBRARY WITH DYNAMIC DLL***


the full config cmd is the following (open3d will be installed in C:/Program Files so you would need admin access for the terminal):

cmake -S .. -B . -G "Visual Studio 16 2019" -A x64 -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_MODULE=OFF -DBUILD_CACHED_CUDA_MANAGER=OFF

Next you build it:

cmake --build . --config Release --target ALL_BUILD

And finally you isntall it:

cmake --build . --config Release --target INSTALL






================

**Cilantro**

- Configure first the project

mkdir build

cd build

cmake -S .. -B . -G "Visual Studio 16 2019" -A x64 

- build it

cmake --build . --config Release --target ALL_BUILD

- and install it

cmake --build . --config Release --target INSTALL
