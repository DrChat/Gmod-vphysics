project "BulletSoftBody"
	
kind "StaticLib"

language "C++"

includedirs {
	"..",
}

files {
	"**.cpp",
	"**.h"
}