#!/bin/bash

cd $DEVEL_DIR/src/hpp_universal_robot/build
cmake -DCMAKE_INSTALL_PREFIX=/home/tpplanning/hpp/install ..
make install

cd $DEVEL_DIR/src
git clone --recursive https://github.com/humanoid-path-planner/hpp-environments

cd $DEVEL_DIR/src/hpp-environments
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/tpplanning/hpp/install ..
make install
