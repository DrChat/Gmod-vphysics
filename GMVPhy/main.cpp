#include "GarrysMod/Lua/Interface.h"

#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

IPhysics *g_pPhysics = NULL;

int lPhysStats(lua_State *state) {
	// 3000 is just a big number, don't worry about it
	for (int i = 0; i < 3000; i++) {
		IPhysicsEnvironment *pEnv = g_pPhysics->GetActiveEnvironmentByIndex(i);
		if (!pEnv)
			break;

		Msg("Environment %d active\n", i);
		Msg("\t%d active objects\n", pEnv->GetActiveObjectCount());
	}

	return 0;
}

// FIXME: The userdata isn't an IPhysicsObject. Figure out what it is.
int lPhysObj_SetGravity(lua_State *state) {
	Msg("top: %d\n", LUA->Top());
	for (int i = 1; i <= LUA->Top(); i++)
		Msg("type: %s\n", LUA->GetTypeName(LUA->GetType(i)));

	return 0;
}

GMOD_MODULE_OPEN() {
	CreateInterfaceFn physFactory = Sys_GetFactory("vphysics");
	if (physFactory)
		g_pPhysics = (IPhysics *)physFactory(VPHYSICS_INTERFACE_VERSION, NULL);

	if (g_pPhysics)
		Msg("Found physics interface!\n");

	// Let's setup our table of functions
	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_GLOB);
		LUA->CreateTable();
			LUA->PushCFunction(lPhysStats); LUA->SetField(-2, "printstats");
		LUA->SetField(-2, "vphysics");
	LUA->Pop();

	// Testing adding new functions to PhysObj lua class
	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_REG);
		LUA->GetField(-1, "PhysObj");
			LUA->PushCFunction(lPhysObj_SetGravity); LUA->SetField(-2, "SetGravity");
		LUA->Pop();
	LUA->Pop();

	return 0;
}

GMOD_MODULE_CLOSE() {
	return 0;
}