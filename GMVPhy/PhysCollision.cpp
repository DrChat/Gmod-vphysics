#include "PhysCollision.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

#include "MiscFuncs.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

IPhysicsCollision1 *	g_pPhysCollision = NULL;

//
// Name: physcollision.CollideSetScale
// Desc: Sets the scale of the specified collision mesh
// Arg1: CPhysCollide|collide|The collision mesh
// Arg2: Vector|scale|The scale to scale by. Vector(1,1,1) is normal scale.
// Ret1: 
//
int lPhysCollisionCollideSetScale(lua_State *state) {
	CPhysCollide *pCollide = Get_PhysCollide(state, 1);
	Vector *scale = Get_Vector(state, 2);
	if (pCollide && scale) {
		g_pPhysCollision->CollideSetScale(pCollide, *scale);
	}

	return 0;
}

//
// Name: physcollision.CollideGetScale
// Desc: Gets the scale of the specified collision mesh
// Arg1: CPhysCollide|collide|The collision mesh
// Ret1: Vector|scale|The scale. Vector(1,1,1) is normal scale.
//
int lPhysCollisionCollideGetScale(lua_State *state) {
	CPhysCollide *pCollide = Get_PhysCollide(state, 1);
	
	if (pCollide) {
		Vector scale;
		g_pPhysCollision->CollideGetScale(pCollide, scale);
		Push_Vector(state, scale);

		return 1;
	}
	
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
			LUA->PushCFunction(lPhysCollisionCollideSetScale);	LUA->SetField(-2, "CollideSetScale");
			LUA->PushCFunction(lPhysCollisionCollideGetScale);	LUA->SetField(-2, "CollideGetScale");
		LUA->SetField(-2, "physcollision");
	LUA->Pop();

	return 0;
}