project "GMVPhy"

language "C++"

kind "SharedLib"

-- Visual studio specific copy command
configuration { "windows", "vs*" }
	if GEN_POSTBUILDCOMMANDS then
		postbuildcommands {
			   'if defined VPHYSICS_GAME_PATH (\n'
			.. ' if exist "%VPHYSICS_GAME_PATH%\garrysmod" (\n'
			.. '  if exist "%VPHYSICS_GAME_PATH%\garrysmod\lua\bin\$(TargetFileName)" (\n'
			.. '   attrib -r "%VPHYSICS_GAME_PATH%\garrysmod\lua\bin\$(TargetFileName)"\n'
			.. '   del "%VPHYSICS_GAME_PATH%\garrysmod\lua\bin\$(TargetFileName)"\n'
			.. '  )\n'
			.. '  \n'
			.. '  copy "$(TargetPath)" "%VPHYSICS_GAME_PATH%\garrysmod\lua\bin\$(TargetFileName)"\n'
			.. ' )\n'
			.. ')\n'
		}
	end

	linkoptions { "/NODEFAULTLIB:\"LIBCMT\"" }

-- Part of a ridiculous hack to get source sdk to build
configuration { "linux", "gmake" }
	buildoptions { "-w", "-fpermissive" }
	defines { "sprintf_s=snprintf", "strcmpi=strcasecmp", "_alloca=alloca", "stricmp=strcasecmp", "_stricmp=strcasecmp", "strcpy_s=strncpy", "_strnicmp=strncasecmp", "strnicmp=strncasecmp", "_snprintf=snprintf", "_vsnprintf=vsnprintf", "_alloca=alloca", "strcmpi=strcasecmp", "NDEBUG", "NO_MALLOC_OVERRIDE" }

configuration {}

defines {"GMMODULE"}

if _PREMAKE_VERSION == "4.4" then
	vpaths {
		["Header Files"] = "**.h",
		["Source Files"] = "**.cpp",
		["Resource Files"] = {"**.rc", "resource.h"},
	}
end

includedirs {
	SDK_DIR,
	SDK_DIR .. "/public",
	SDK_DIR .. "/public/tier0",
	SDK_DIR .. "/public/tier1",
	GM_MODULEBASE,
}

files {
	"**.cpp",
	"**.h"
}