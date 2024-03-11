
# ROH Serial API Example

Scan ans set ID for ROH.

Ubuntu 22.04
gcc version 11.4.0

## Install serial lib

### Linux

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

### Windows

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

## Compile

```BASH
source /usr/local/serial/setup.bash  # for serial lib

cd /path/to/ohand_serial/scan_and_set_id
mkdir build && cd build
cmake ..
make
```

## Run

```BASH
./simple_control /dev/ttyx           # Run, replace ttyx with real port name
```
