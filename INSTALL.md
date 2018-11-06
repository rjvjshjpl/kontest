# Build procedure

Getting the proper build environment for KLEE set up is a pain.
Here is a step-by-step procedure, based on a [tutorial](https://github.com/epsallida/install-klee) by Eirini Psallida.
This is a totally local build, not affecting directories outside of wherever you start (and not requiring any special permissions).
It has been tested on Ubuntu 14.04 and macOS 10.12, and should work on other platforms supported by KLEE (e.g. other varieties of Linux).

## 1. Build LLVM

Kontest requires LLVM 3.6 or 3.7 (at the moment).
It is known to work with LLVM 3.6.2 and 3.7.1 -- versions in between will probably work as well.
The instructions below use LLVM 3.6.2 as an example.

To build LLVM you need a whole pile of prerequisites, described [here](http://llvm.org/docs/GettingStarted.html#requirements).
You may be able to get these with something like `apt-get build-dep llvm`.

Download the sources for LLVM 3.6.2 and various components:
```
wget http://releases.llvm.org/3.6.2/llvm-3.6.2.src.tar.xz
wget http://releases.llvm.org/3.6.2/cfe-3.6.2.src.tar.xz
wget http://releases.llvm.org/3.6.2/libcxx-3.6.2.src.tar.xz
tar -xf llvm-3.6.2.src.tar.xz
tar -xf cfe-3.6.2.src.tar.xz
tar -xf libcxx-3.6.2.src.tar.xz
mv cfe-3.6.2.src/ llvm-3.6.2.src/tools/clang
mv libcxx-3.6.2.src/ llvm-3.6.2.src/projects/libcxx
```

Now we build LLVM (this may take a while).
If you have an old version of Python installed, CMake may latch onto it instead of the newer version and the configure will fail.
You can add `-DPYTHON_EXECUTABLE=/PATH/TO/PYTHON` as appropriate.
```
cd llvm-3.6.2.src
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DLLVM_TARGETS_TO_BUILD=host ..
make -j `nproc`
make -j `nproc` check-all
cd ../..
```

## 2. Build MiniSat

The STP people maintain a very slightly modified version of MiniSat.
There doesn't seem to be an official release, so we have to just clone the GitHub repository.
```
git clone --depth 1 https://github.com/stp/minisat.git
cd minisat
make -j `nproc`
cd ..
```

## 3. Build STP 2.2.0

STP requires a non-ancient version of _flex_ but doesn't check the version itself.
You can check the version with `flex --version`.
Version 2.5.4 does not work, while 2.5.35 does (and probably newer versions too).
I haven't tested versions in between, but if when building STP you get a link error about an undefined reference to `cvclex_destroy`, you need a newer version.
To install version 2.5.35, you can do:
```
wget https://github.com/westes/flex/archive/flex-2-5-35.tar.gz
tar -xf flex-2-5-35.tar.gz
cd flex-flex-2-5-35
./autogen.sh
./configure --prefix=/SOMEWHERE/WHOSE/bin/SUBDIRECTORY/IS/ON/YOUR/PATH
make
make install
cd ..
```

To build STP:
```
wget https://github.com/stp/stp/archive/stp-2.2.0.tar.gz
tar -xf stp-2.2.0.tar.gz
cd stp-stp-2.2.0
mkdir build
cd build

cmake \
  -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_STATIC_BIN=ON \
  -DENABLE_PYTHON_INTERFACE=OFF \
  -DCMAKE_BUILD_TYPE="Release" \
  -DTUNE_NATIVE=ON \
  -DMINISAT_LIBRARY=../../minisat/build/release/lib/libminisat.a \
  -DMINISAT_INCLUDE_DIR=../../minisat \
  ..

make -j `nproc`
cd ../..
```

## 4. Build Kontest

If you have the CLion IDE, you can use a premade project (which expects you to have built `kontest-uclibc` -- see below).
```
cd kontest
cp -r idea .idea
```
Then launch CLion, pick "Open Project", and select the `kontest` folder.

Otherwise, you can build Kontest manually using CMake:
```
cd kontest
mkdir Debug
cd Debug

cmake \
  -DUSE_CMAKE_FIND_PACKAGE_LLVM=OFF \
  -DLLVM_CONFIG_BINARY=../../llvm-3.6.2.src/build/bin/llvm-config \
  -DSTP_DIR=../../stp-stp-2.2.0/build \
  -DENABLE_SOLVER_STP=ON \
  -DENABLE_UNIT_TESTS=OFF \
  -DENABLE_SYSTEM_TESTS=OFF \
  -DENABLE_KLEE_ASSERTS=ON \
  ..
  
make -j `nproc`
```
The KLEE binary should then be available in `bin`.
For a Release build, add `-DCMAKE_BUILD_TYPE=Release` (and optionally change `ENABLE_KLEE_ASSERTS` to `OFF`).

## (5.) Build uClibc

It is possible to test applications that use the C standard library with Kontest (this is not true if they use the C++ standard library, unfortunately).
To do this, you must build a modified version of uClibc.
This is only possible on Linux platforms (as far as I know), and in particular it does not work on macOS.
Proceed as follows:
```
<Get a copy of the kontest-uclibc repository>
cd kontest-uclibc
./configure --make-llvm-lib --with-llvm-config ../llvm-3.6.2.src/build/bin/llvm-config
make -j `nproc`
```
If you get errors early in the make process (when building `conf`), it may be that your system's version of _glibc_ is too old for `clang` to handle (_glibc_ 2.5 for example assumes the _gnu89_ inline semantics, leading to "multiple definition" errors for `fstat`, `gnu_dev_major`, etc. when used with `clang`).
Try `make clean`, then add `--with-hostcc=gcc` to the `configure` command and rebuild.

Once you have built uClibc, rebuild Kontest adding the following CMake arguments: `-DENABLE_KLEE_UCLIBC=ON -DKLEE_UCLIBC_PATH=../../kontest-uclibc`.

## Runtime Variations: 32-bit, old DWARF formats

If you want to test 32-bit applications with Kontest, you need to build 32-bit versions of the KLEE runtime libraries (otherwise when running KLEE you'll get warnings about linking modules with conflicting data layouts, and probably spurious errors).
Simply use the cmake option `-DKLEE_RUNTIME_32BIT=ON`.
If you're using uClibc, you also need to build a 32-bit version of that.
Run `configure` as above, but add `KLEE_CFLAGS=-m32` to the `make` command.

Also, KLEE normally uses the system `malloc` to allocate memory for the test program, and since KLEE is 64-bit this won't work with 32-bit programs (which expect 32-bit pointers).
So you need to turn on KLEE's custom memory allocator with the command-line option `--allocate-determ`.
The allocator is extremely simplistic and does not reclaim memory freed by the program (until it terminates -- this is because in vanilla KLEE freed memory might still be accessed from another state).
So you may need to raise the amount of memory the allocator can use with `--allocate-determ-size` (see the usage message).
When the allocator runs out memory KLEE will print a warning, and with the option `--exit-on-alloc-failure` it will terminate.
Without this option KLEE simply returns `0` to the program as if `malloc` failed, but this can cause crashes even if the program checks the return value of every call to `malloc`, since stack allocation is handled the same way (i.e. the __alloca__ instruction will return `0`, causing an out-of-bounds memory error later).

If when using KLEE you get errors about 'Dwarf Version' flags having conflicting values, then you built the runtime and/or uClibc with a different version of the DWARF debug symbol format than the program being tested.
To fix the runtime, add `-DKLEE_RUNTIME_DEBUG_FLAGS=-gdwarf-2` (or whatever is appropriate) when configuring Kontest.
To fix uClibc, first do a `make clean`.
Then run `configure` as above and do `make menuconfig`.
This will bring up a menu: go to "uClibc development/debugging options" and change the "Extra CFLAGS used for debug builds" from `-O0 -g3` to `-O0 -gdwarf-2` or whatever version you need (note that `-g3` forces at least DWARF version 4).
Save the changes and rebuild.

# Unusual builds

__N.B.:__ _This section is mainly to document different types of KLEE builds for the developers. These builds may not work for Kontest. In particular, KLEE's old autoconf system has not been updated to build the Kontest runtime library._

## Using autoconf

The KLEE CMake system is experimental and doesn't work for some of the more complicated builds below (as far as I know).
So if needed you can use the autoconf-based system as follows:
```
cd kontest

./configure \
  LDFLAGS=-L`cd ../minisat/build/release/lib; pwd` \
  --with-llvm=`cd ../llvm-3.4.2.src; pwd` \
  --with-llvmcc=`cd ../llvm-3.4.2.src/Release+Asserts/bin; pwd`/clang \
  --with-llvmcxx=`cd ../llvm-3.4.2.src/Release+Asserts/bin; pwd`/clang++ \
  --with-stp=`cd ../stp-stp-2.2.0/build; pwd`

make -j `nproc`
```

The KLEE binary should then be available at `Release+Asserts/bin`.

## Extensions

Notes on more complicated builds.

### Z3

KLEE can use Z3 as its backend solver.
The easiest way is to install the appropriate binaries for your system.
Version 4.5.0 works, and newer versions will probably work too.
If using CMake, hopefully it will find them (I couldn't test since I don't have root access, and a local install didn't work when I tried it quickly).

Alternatively, you can use the autoconf method and build Z3 from source:
```
wget https://github.com/Z3Prover/z3/archive/z3-4.5.0.tar.gz
tar -xf z3-4.5.0.tar.gz
cd z3-z3-4.5.0
python scripts/mk_make.py --prefix=`pwd`
cd build
make -j `nproc`
make install
cd ../..
```
Then add the following to the `configure` command for KLEE:
```
--with-z3=`cd ../z3-z3-4.5.0; pwd`
```
and do the following after compiling KLEE:
```
cp ../z3-z3-4.5.0/lib/libz3.so Release+Asserts/lib/
export LD_LIBRARY_PATH+=`cd Release+Asserts/lib; pwd`
```

### CryptoMiniSat

To build STP with CryptoMiniSat, first build the latter:
```
wget https://github.com/msoos/cryptominisat/archive/5.0.1.tar.gz
tar -xf 5.0.1.tar.gz
cd cryptominisat-5.0.1
mkdir build
cd build

cmake \
  -DSTATICCOMPILE=ON \
  -DENABLE_PYTHON_INTERFACE=OFF \
  -DNOVALGRIND=ON \
  -DNOZLIB=ON \
  -DNOM4RI=ON \
  -DONLY_SIMPLE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=. \
  ..

make -j `nproc`
cd ../..
```

Then add the following before the `cmake` command for STP:
```
cryptominisat5_DIR=../../cryptominisat-5.0.1/build
```

Finally, add the following to the `LDFLAGS` when configuring KLEE:
```
-L../../cryptominisat-5.0.1/build/lib
```
I haven't tried this when building KLEE with CMake.

_NOTE: When used as a library, STP does not use CryptoMiniSat by default!
KLEE must be modified to call `vc_setInterfaceFlags(vc, CMS4, 0)`._
