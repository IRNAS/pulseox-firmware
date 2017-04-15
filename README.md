# Pulse Oximeter Firmware

[![Build Status](https://travis-ci.org/IRNAS/pulseox-firmware.svg?branch=master)](https://travis-ci.org/IRNAS/pulseox-firmware)

The following packages are needed for compilation:
* cmake
* gcc-arm-embedded

To compile:
```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/arm-none-eabi.cmake ..
make
```

