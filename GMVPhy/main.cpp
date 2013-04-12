#include "GarrysMod/Lua/Interface.h"

#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// A note about this module:
// The structures we're using here are internal to Garry's Mod, and they may change!
// Do not release this module.

using namespace GarrysMod::Lua;

IPhysics1 *g_pPhysics = NULL;

IPhysicsObject1 *Get_PhysObj(lua_State *state, int stackPos) {
	LUA->CheckType(stackPos, Type::PHYSOBJ);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	return (IPhysicsObject1 *)ud->data;
}

Vector *Get_Vector(lua_State *state, int stackPos) {
	LUA->CheckType(stackPos, Type::VECTOR);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	return (Vector *)ud->data;
}

void Push_Vector(lua_State *state, const Vector &vec) {
	LUA->PushSpecial(SPECIAL_GLOB);
		LUA->GetField(-1, "Vector");
		LUA->PushNumber(vec.x);
		LUA->PushNumber(vec.y);
		LUA->PushNumber(vec.z);
		LUA->Call(3, 1);
	LUA->Remove(-2);
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

//
// Name: PhysObj:GetGravity
// Desc: Gets the object's current gravity. Will return the same gravity as the environment if you didn't set it previously.
// Arg1:
// Ret1: Vector|gravityVec|The gravity vector
//
int lPhysObjGetGravity(lua_State *state) {
	IPhysicsObject1 *pObject = Get_PhysObj(state, 1);
	Vector grav = pObject->GetGravity();

	Push_Vector(state, grav);

	return 1;
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
			LUA->PushCFunction(lPhysObjGetGravity); LUA->SetField(-2, "GetGravity");
		LUA->Pop();
	LUA->Pop();

	return 0;
}

GMOD_MODULE_CLOSE() {
	return 0;
}