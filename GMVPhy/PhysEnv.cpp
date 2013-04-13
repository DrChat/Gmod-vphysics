#include "PhysEnv.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

#include "MiscFuncs.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

int Init_PhysEnv(lua_State *state) {
	LUA->PushSpecial(SPECIAL_GLOB);
		LUA->GetField(-1, "physenv");
			
	LUA->Pop(2);
}