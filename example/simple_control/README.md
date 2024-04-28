
# ROH Serial API Example

Example for ROH simple control.

## 1. Linux

Ubuntu 22 + gcc version 11

### 1.1. Install serial lib

```BASH
cd ~
git clone https://github.com/wjwwood/serial
cd serial
vi ~/serial/Makefile
```

Modify CMAKE_INSTALL_PREFIX as following:

```TXT
#CMAKE_FLAGS := -DCMAKE_INSTALL_PREFIX=/tmp/usr/local
CMAKE_FLAGS := -DCMAKE_INSTALL_PREFIX=/usr/local/serial
```

Then compile:

```BASH
make
sudo make install
```

### 1.2. Compile

```BASH
source /usr/local/serial/setup.bash  # for serial lib

cd /path/to/ohand_serial/simple_control
mkdir build && cd build
cmake ..
make
```

### 1.3. Run

```BASH
./simple_control /dev/ttyx           # Run, replace ttyx with real port name
```

## 2. Windows

### 2.1. Install serial lib

Clone https://github.com/wjwwood/serial and open file visual_studio/visual_studio.sln in repository dir then compile.
Put files to the same partition as your ohand_serial_sdk, e.g., d:\
Make dirs/files look as following:

```TXT
d:\serial
├─include
│  └─serial
│      │  serial.h
│      │  v8stdint.h
│      │
│      └─impl
│              unix.h
│              win.h
│
└─lib
    ├─Debug
    │      serial.idb
    │      serial.lib
    │      serial.pdb
    │
    └─Release
            serial.lib
            serial.pdb
```

### 2.2. Compile

```BATCH

cd path_to_project
md build
cd build
cmake ..

REM Compiles debug version
cmake --build . --config Debug

REM Compiles release version
cmake --build . --config Release

```

### 2.3. Run

```BATCH
cd path_to_project/build/Debug
REM Or 'Release'

REM Run, replace COMx with real port name
simple_control COMx
```
