--[[-----------
-- Options
-------------]]

-- SRCDS dir
newoption {
	trigger		= "srcdsbindir",
	value		= "path",
	description	= "Source Dedicated Server engine bin directory (required on linux)"
}

SRCDS_BIN_DIR = _OPTIONS["srcdsbindir"]

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