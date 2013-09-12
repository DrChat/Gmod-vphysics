#!/usr/bin/env sh

# Settings
WORKING_DIR=$(pwd)
BULLET_PROJ_DIR=$WORKING_DIR/bullet/proj

# Build bullet
cd "$BULLET_PROJ_DIR"
./premake4 gmake
cd gmake
#make clean
make -j16

# Build vphysics
cd "$WORKING_DIR/src"
#make clean
make -j16


cd "$WORKING_DIR"
