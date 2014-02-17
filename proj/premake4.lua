solution "vphysics"

if _ACTION == "vs2010" or _ACTION == "vs2008" then
	-- Enable multi-processor compilation
	buildoptions { "/MP"  }
end

act = ""

if _ACTION then
	act = _ACTION
end

-- Options
dofile("options.lua")

--[[---------------------------------
-- Main configuration properities
-----------------------------------]]
configurations { "Release", "Debug" }

configuration "Release"
	defines { "RELEASE=1", "NDEBUG=1", "_RELEASE=1" }
	if os.is( "linux" ) then
		defines { "_LINUX=1", "__linux__=1", "LINUX=1" }
	end
	
	-- TODO: When supported, add NoIncrementalLink flag
	flags { "OptimizeSpeed", "EnableSSE", "StaticRuntime", "NoMinimalRebuild", "FloatFast" }
	targetdir( "../build/bin/" .. os.get() .. "/release" )
	
configuration "Debug"
	defines { "_DEBUG=1" }
	if os.is("linux") then
		defines { "_LINUX=1", "__linux__=1", "LINUX=1" }
	end
	
	flags { "Symbols", "StaticRuntime", "NoMinimalRebuild", "FloatFast" }
	targetdir("../build/bin/" .. os.get() .. "/debug")

configuration {}

-- Only support 32 bit builds
configuration { "linux", "gmake" }
	buildoptions { "-fPIC", "-m32" }
	linkoptions { "-m32" }
	
configuration {}

location( "./" .. act )

--[[--------------
-- Projects
----------------]]
-- Hardcoded path now because this won't ever change
SDK_DIR = "../thirdparty/sourcesdk/mp/src"
GM_MODULEBASE = "../thirdparty/gmmodulebase/include"

include "../GMVPhy"

if os.is("linux") and not SRCDS_BIN_DIR then
	print "Fatal: SRCDS bin dir was not defined (which is required for linux builds)\nCannot generate vphysics project files" 
else
	include "../src"
end

include "../bullet/src"