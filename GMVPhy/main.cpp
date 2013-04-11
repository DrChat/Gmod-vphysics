#include "GarrysMod/Lua/Interface.h"

#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

// A note about this module:
// The structures we're using here are internal to Garry's Mod, and they may change!
// Do not release this module.

IPhysics1 *g_pPhysics = NULL;

IPhysicsObject1 *Get_PhysObj(lua_State *state, int stackPos) {
	int type = LUA->GetType(stackPos);
	if (type == GarrysMod::Lua::Type::PHYSOBJ) {
		// GetUserData for PhysObj returns a pointer to a pointer of the physics object. (Really returns an unknown struct)
		return *(IPhysicsObject1 **)LUA->GetUserdata(stackPos);
	} else {
		char str[1024];
		sprintf_s(str, "User data at stack position %d is not a PhysObj!\n", stackPos);
		LUA->ThrowError(str);
	}

	return NULL;
}

Vector *Get_Vector(lua_State *state, int stackPos) {
	int type = LUA->GetType(stackPos);
	if (type == GarrysMod::Lua::Type::VECTOR) {
		return *(Vector **)LUA->GetUserdata(stackPos);
	} else {
		char str[1024];
		sprintf_s(str, "User data at stack position %d is not a Vector!\n", stackPos);
		LUA->ThrowError(str);
	}

	return NULL;
}

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

//
// Name: PhysObj:SetGravity
// Desc: Sets an object's gravity. You must disable the environment's gravity with EnableGravity(false) first!
// Arg1: Vector|gravityVec|The gravity vector
// Ret1:
//
int lPhysObjSetGravity(lua_State *state) {
	IPhysicsObject1 *pObject = Get_PhysObj(state, 1);
	Vector *pVec = Get_Vector(state, 2);

	if (pVec)
		pObject->SetGravity(*pVec);

	return 0;
}

GMOD_MODULE_OPEN() {
	CreateInterfaceFn physFactory = Sys_GetFactory("vphysics");
	if (physFactory) {
		g_pPhysics = (IPhysics1 *)physFactory(VPHYSICS_INTERFACE_VERSION, NULL);
	}

	if (g_pPhysics) {
		Msg("Found physics interface!\n");
	}

	// Let's setup our table of functions
	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_GLOB);
		LUA->CreateTable();
			LUA->PushCFunction(lPhysStats); LUA->SetField(-2, "printstats");
		LUA->SetField(-2, "vphysics");
	LUA->Pop();

	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_REG);
		LUA->GetField(-1, "PhysObj");
			LUA->PushCFunction(lPhysObjSetGravity); LUA->SetField(-2, "SetGravity");
		LUA->Pop();
	LUA->Pop();

	return 0;
}

GMOD_MODULE_CLOSE() {
	return 0;
}