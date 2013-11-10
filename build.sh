#!/usr/bin/env sh

# Settings
WORKING_DIR=$(pwd)
BULLET_PROJ_DIR=$WORKING_DIR/bullet/proj

init() {
	NUM_THREADS="1"
	CLEAN=""

	while test $# -gt 0; do
		case "$1" in
			"-numthreads")
				NUM_THREADS="$2"
				shift;;
			"-clean")
				CLEAN="yes"
				;;
		esac
		shift # Shifts command line arguments var or something whatever
	done
}

usage() {
	echo "Usage: $0 <num threads>"
	exit 1
}

build_bullet() {
	if test -n $BULLET_PROJ_DIR; then
		cd "$BULLET_PROJ_DIR"
		./premake4 gmake
		cd gmake

		if test -n "$CLEAN"; then
			make clean
		fi

		make -j$NUM_THREADS
	fi
}

build_vphysics() {
	cd "$WORKING_DIR/src"

	if test -n "$CLEAN"; then
		make clean
	fi

	make -j$NUM_THREADS
}

init

build_bullet
build_vphysics

cd "$WORKING_DIR"
