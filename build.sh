#!/usr/bin/env sh

# Settings
WORKING_DIR=$(pwd)
BULLET_PROJ_DIR=$WORKING_DIR/bullet/proj

NUM_THREADS="$1"

usage() {
	echo "Usage: $0 <num threads>"
	exit 1
}

build_bullet() {
	if test -n $BULLET_PROJ_DIR
	then
		cd "$BULLET_PROJ_DIR"
		./premake4 gmake
		cd gmake
		make -j$NUM_THREADS
	fi
}

build_vphysics() {
	cd "$WORKING_DIR/src"
	make -j$NUM_THREADS
}

if [ $# -eq 0 ];
then
	usage
fi

build_bullet
build_vphysics

cd "$WORKING_DIR"
