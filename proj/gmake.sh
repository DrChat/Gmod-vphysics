#!/bin/bash

# REMOVE THIS AFTER YOU CONFIGURE THE FILE
echo "Configure the script first!"
exit 1

./premake4 --sdkdir="x/SourceMod/src" --srcdsbindir="x/srcds/orangebox/bin" gmake
