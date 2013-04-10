Gmod-vphysics
=============

Replacement vphysics module for Garry's Mod

## Usage

### Users
#### Download
You can download vphysics [here](http://peniscorp.com/vphysics/) (courtesy of Gran PC)
#### Installation
1. Copy vphysics.dll to the engine bin directory (same folder as the one that contains hl2.exe)
2. In the steam directory, create a file called "steam.cfg" and copy this code into it:

`BootStrapperInhibitAll=enable`
`MinFootprintAutoRefresh=disable`
3. Restart steam and launch Garry's Mod.


### Developers
#### Libraries Required
* Source SDK
* SDL (Optional, used for disabled by default part of debug drawer)

#### Automatic Copying
You can set the environmental variable "VPHYSICS_GAME_PATH" to the directory that contains hl2.exe and vphysics will automatically be copied over.

#### Pull Requests
Please make the code in your pull requests look neat. I will only be pulling from them if they're in the same coding style that we're using. If it's a pull request for bullet code, please use the same coding style as bullet.
