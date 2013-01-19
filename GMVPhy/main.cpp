#include "GarrysMod/Lua/Interface.h"

GMOD_MODULE_OPEN() {
	// Let's setup our table of functions
	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_GLOB);
		LUA->CreateTable();
			// FYI value = key
			LUA->PushString("Hi"); LUA->SetField(-2, "hello");
		LUA->SetField(-2, "vphysics");
	LUA->Pop();

	return 0;
}

GMOD_MODULE_CLOSE() {
	return 0;
}