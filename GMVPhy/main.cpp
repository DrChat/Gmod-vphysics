#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

#include "PhysObj.h"
#include "PhysCollision.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

IPhysics1 *g_pPhysics = NULL;

int lPhysStats(lua_State *state) {
	for (int i = 0; i < g_pPhysics->GetNumActiveEnvironments(); i++) {
		IPhysicsEnvironment *pEnv = g_pPhysics->GetActiveEnvironmentByIndex(i);
		if (!pEnv)
			break;

		Msg("Environment %d active\n", i);
		Msg("\t%d active objects\n", pEnv->GetActiveObjectCount());
	}

	return 0;
}

GMOD_MODULE_OPEN() {
	CreateInterfaceFn physFactory = Sys_GetFactory("vphysics");
	if (physFactory) {
		g_pPhysics = (IPhysics1 *)physFactory(VPHYSICS_INTERFACE_VERSION, NULL);
	}

#ifdef _DEBUG
	if (g_pPhysics) {
		Msg("Found physics interface!\n");
	}
#endif

	// Let's setup our table of functions
	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_GLOB);
		LUA->CreateTable();
			LUA->PushCFunction(lPhysStats); LUA->SetField(-2, "printstats");
		LUA->SetField(-2, "vphysics");
	LUA->Pop();

	Init_PhysObj(state);
	Init_PhysCollision(state);

	return 0;
}

GMOD_MODULE_CLOSE() {
	return 0;
}