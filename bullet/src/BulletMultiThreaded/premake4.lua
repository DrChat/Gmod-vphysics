project "BulletMultiThreaded"
	
kind "StaticLib"

language "C++"

includedirs {
	"..",
}

files {
	"**.cpp",
	"**.h"
}

excludes {
	"GpuSoftBodySolvers/**"
}