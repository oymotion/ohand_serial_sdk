
# ROH Serial API Example

Example for ROH simple control.

## 1. Linux

Ubuntu 20 + gcc version 11

### 1.1. Install serial lib

```BASH
sudo apt install catkin
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
sudo make && sudo make install
```

### 1.2. Install PCAN-Linux-Driver

Download from https://www.peak-system.com/fileadmin/media/linux/index.php

```bash
tar -xzf peak-linux-driver-8.20.0.tar.gz
cd peak-linux-driver-8.20.0
sudo make && sudo make install
```

### 1.3. Install PCAN-Basic lib

Download from https://www.peak-system.com/PCAN-Basic-Linux.433.0.html?&L=1

```BASH
tar -xzf PCAN-Basic_Linux-4.10.0.4.tar.gz
cd PCAN-Basic_Linux-4.10.0.4/libpcanbasic/pcanbasic
make clean
sudo make && sudo make install
```

### 1.4. Compile

Modify PORT_TYPE according to protocol type in main.cpp:

- RS485: `PORT_TYPE = PORT_TYPE_UART`
- CAN: `PORT_TYPE = PORT_TYPE_CAN`

```BASH
source /usr/local/serial/setup.bash  # for serial lib

cd path/to/project
mkdir build && cd build
cmake ..
sudo make
```

### 1.5. Run

```BASH
cd path/to/project/build

REM Run serial, replace ttyx with real port name
sudo chomod +x /dev/ttyx  # replace ttyx with real port name
./force_control /dev/ttyx

REM Run can, replace x with number from 1 - 16
sudo chomod +x /dev/pcan-usb
./force_control x
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

### 2.2 Install PCAN-Basic lib

Download from https://www.peak-system.com/PCAN-Basic.126.0.html?&L=1 

Put files to the same partition as your ohand_serial_sdk, e.g., d:\

Make dirs/files look as d:\PCAN-Basic

### 2.3. Compile

Modify PORT_TYPE according to protocol type in main.cpp:

- RS485: `PORT_TYPE = PORT_TYPE_UART`
- CAN: `PORT_TYPE = PORT_TYPE_CAN`

```BATCH

cd path\to\project
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
cd path\to\project\build\Debug
REM Or 'Release'

REM Run serial, replace COMx with real port name
force_control COMx

REM Run can, replace x with number from 1 - 16
force_control  x
```

## 3. User Guide

After the program starts, you can press following keys to operate: 

| key | Description                       |
| --- | ----------------------------------|
| q/Q | start thumb finger force control  |
| w/W | start index finger force control  |
| e/E | start middle finger force control |
| r/R | start ring finger force control   |
| t/T | start little finger force control |
| a/A | stop thumb finger force control   |
| s/S | stop index finger force control   |
| d/D | stop middle finger force control  |
| f/F | stop ring finger force control    |
| g/G | stop little finger force control  |