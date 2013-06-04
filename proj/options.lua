--[[-----------
-- Options
-------------]]
-- Source SDK Dir
newoption {
	trigger		= "sdkdir",
	value		= "path",
	description	= "Source SDK Directory (root folder should be src)"
}

SDK_DIR = _OPTIONS["sdkdir"]

-- SRCDS dir
newoption {
	trigger		= "srcdsbindir",
	value		= "path",
	description	= "Source Dedicated Server engine bin directory"
}

SRCDS_BIN_DIR = _OPTIONS["srcdsbindir"]

-- Gmod Module Base (optional)
newoption {
	trigger		= "gmmodulebase",
	value		= "path",
	description	= "path to the garry's mod module base (optional, for GMVPhy)"
}

GMMODULEBASE_DIR = _OPTIONS["gmmodulebase"]
USE_GMVPHY = true

if not GMMODULEBASE_DIR then
	USE_GMVPHY = false
end

-- Generate post build commands for VS projects
newoption {
	trigger		= "genpostbuildcommand",
	value		= "true",
	description	= "Allow post build command generation (VS only)",
	allowed		= {
		{ "true",	"true" },
		{ "false",	"false" },
	}
}

GEN_POSTBUILDCOMMANDS = false
if _OPTIONS["genpostbuildcommand"] == "true" then
	GEN_POSTBUILDCOMMANDS = true
end