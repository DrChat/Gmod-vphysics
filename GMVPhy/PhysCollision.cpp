#include "PhysCollision.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

#include "MiscFuncs.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

IPhysicsCollision32 *	g_pPhysCollision = NULL;

int lPhysConvex__gc(lua_State *state) {
	

	return 0;
}

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

//
// Name: physcollision.ConvertConvexToCollide
// Desc: Converts table of convexes to collide
// Arg1: Table|convexes|The table of convex meshes {{(optional) pos=Vector(0,0,0), (optional) ang=Angle(0,0,0), convex=nil}, ...}
// Ret1: CPhysCollide|collide|The collision mesh.
//
int lPhysCollisionConvertConvexToCollide(lua_State *state) {
	return 0;
}

//
// Name: physcollision.CylinderToConvex
// Desc: Converts AABB to a cylinder
// Arg1: Vector|mins|
// Arg2: Vector|maxs|
// Ret1: CPhysConvex|convex|The convex mesh.
//
int lPhysCollisionCylinderToConvex(lua_State *state) {
	Vector *pMins = Get_Vector(state, 1);
	Vector *pMaxs = Get_Vector(state, 2);

	if (pMins && pMaxs) {
		CPhysConvex *pConvex = g_pPhysCollision->CylinderToConvex(*pMins, *pMaxs);
	}

	return 0;
}

int Init_PhysCollision(lua_State *state) {
	CreateInterfaceFn physFactory = Sys_GetFactory("vphysics");
	if (physFactory)
		g_pPhysCollision = (IPhysicsCollision32 *)physFactory(VPHYSICS_COLLISION_INTERFACE_VERSION, NULL);

	if (!g_pPhysCollision) {
		Warning("Couldn't load the physics collision factory!\n");
		return 1;
	}

	LUA->CreateMetaTableType("PhysConvex", CustomTypes::TYPE_PHYSCONVEX);
		
	LUA->Pop();

	LUA->PushSpecial(SPECIAL_GLOB);
		LUA->CreateTable();
			LUA->PushCFunction(lPhysCollisionCollideSetScale);	LUA->SetField(-2, "CollideSetScale");
			LUA->PushCFunction(lPhysCollisionCollideGetScale);	LUA->SetField(-2, "CollideGetScale");
		LUA->SetField(-2, "physcollision");
	LUA->Pop();

	return 0;
}