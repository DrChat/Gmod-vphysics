Gmod-vphysics
=============

Replacement vphysics module for the source engine (formerly just garry's mod)

## Users
#### Download
You can download vphysics [here](http://peniscorp.com/vphysics/) (courtesy of Gran PC)
#### Installation
1. Copy vphysics.dll to the engine bin directory (parent folder is the one that contains hl2.exe, e.g. \<username\>/garrysmod/bin)
2. Run the game.

You may have to repeat these steps if the game updates!

#### Donate
Support the developer!

[![Donate Via Paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](http://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=3UA358MF6QLZE)

<sub>If you fork this repository you may want to change this to your paypal account or remove it.</sub>

## Developers
#### Libraries Required
* Absolutely nothing! Everything needed included in ./thirdparty

#### Git Submodules
This library uses git submodules. In order to set those up, after cloning the repository, issue these commands:

```
git submodule init

# And to update submodules:
git submodule update
```

Alternatively, you can also clone the repository with the --recursive switch, as so:
```
git clone --recursive <url>
```

Submodules will need to be compiled separately from vphysics if required (atleast until a script is setup)

#### Windows Setup
* Visual studio 2012 projects provided
* - You must configure the library paths manually (sorry!)
* Optionally, premake scripts are provided in the proj folder

#### Linux Setup
* You need to have the game you're building for installed, as vphysics needs to dynamically link to some modules. Configure the path in ./src/Makefile
* You can use the build.sh provided in the root directory to compile vphysics. Optional arguments are -numthreads \<count\> and -clean (make clean before make)

#### Automatic Copying
On Windows, you can set the environmental variable "VPHYSICS_GAME_PATH" to the directory that contains hl2.exe and vphysics will automatically be copied over.

#### Pull Requests
Please make the code in your pull requests look neat, and have the commits be granular (handle one thing at a time). They will only be pulled if they're in the same coding style as the surrounding code (don't make your code stand out!)
