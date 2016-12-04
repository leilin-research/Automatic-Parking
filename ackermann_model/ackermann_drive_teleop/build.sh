#!/bin/sh
rm -rf build
rm -rf install
mkdir -p build
mkdir -p install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
make install
cd -
