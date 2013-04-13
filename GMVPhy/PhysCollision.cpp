#include "PhysCollision.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

#include "MiscFuncs.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

IPhysicsCollision1 *	g_pPhysCollision = NULL;

int lPhysCollisionCollideSetScale(lua_State *state) {
	return 0;
}

int lPhysCollisionCollideGetScale(lua_State *state) {
	return 0;
}

int Init_PhysCollision(lua_State *state) {
	CreateInterfaceFn physFactory = Sys_GetFactory("vphysics");
	if (physFactory)
		g_pPhysCollision = (IPhysicsCollision1 *)physFactory(VPHYSICS_COLLISION_INTERFACE_VERSION, NULL);

	if (!g_pPhysCollision) {
		Warning("Couldn't load the physics collision factory!\n");
		return 1;
	}

	LUA->PushSpecial(SPECIAL_GLOB);
		LUA->CreateTable();
			LUA->PushCFunction(lPhysCollisionCollideSetScale); LUA->SetField(-2, "CollideSetScale");
			LUA->PushCFunction(lPhysCollisionCollideGetScale); LUA->SetField(-2, "CollideGetScale");
		LUA->SetField(-2, "physcollision");
	LUA->Pop();

	return 0;
}