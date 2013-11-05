Gmod-vphysics
=============

Replacement vphysics module for Garry's Mod

## Usage

### Users
#### Download
You can download vphysics [here](http://peniscorp.com/vphysics/) (courtesy of Gran PC)
#### Installation
1. Copy vphysics.dll to the engine bin directory (same folder as the one that contains hl2.exe)
2. Run Garry's Mod.

You may have to repeat these steps after updating garry's mod!


### Developers
#### Libraries Required
* Source SDK
* SDL (Optional, used for disabled by default part of debug drawer)

#### Windows Setup
* Visual studio 2012 projects provided
* - You must configure the library paths manually (sorry!)
* Optionally, premake scripts are provided in the proj folder

#### Linux Setup
* You can use the build.sh provided in the root directory to compile vphysics. Be sure to setup src/Makefile before you do!

#### Automatic Copying
You can set the environmental variable "VPHYSICS_GAME_PATH" to the directory that contains hl2.exe and vphysics will automatically be copied over.

#### Pull Requests
Please make the code in your pull requests look neat. I will only be pulling from them if they're in the same coding style that we're using. If it's a pull request for bullet code, please use the same coding style as bullet.
