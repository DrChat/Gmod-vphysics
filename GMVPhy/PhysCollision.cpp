#include "PhysCollision.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"
#include <tier1.h>

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

	g_pPhysCollision->CollideSetScale(pCollide, *scale);

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

	Vector scale;
	g_pPhysCollision->CollideGetScale(pCollide, scale);
	Push_Vector(state, scale);

	return 1;
}

int lPhysCollisionDestroyCollide(lua_State *state) {
	CPhysCollide *pCollide = Get_PhysCollide(state, 1);
	g_pPhysCollision->DestroyCollide(pCollide);

	LUA->PushNil();
	LUA->SetMetaTable(1);
	return 0;
}

//
// Name: physcollision.ConvertConvexesToCollide
// Desc: Converts table of convexes to collide. It's safe to discard the convexes after this call, but DO NOT free them!
// Arg1: Table|convexes|The table of convex meshes {{(optional) pos=Vector(0,0,0), (optional) ang=Angle(0,0,0), convex=nil}, ...}
// Ret1: CPhysCollide|collide|The collision mesh.
//
int lPhysCollisionConvertConvexesToCollide(lua_State *state) {
	LUA->CheckType(1, Type::TABLE);

	CUtlVector<CPhysConvex *> convexes;
	LUA->PushNil(); // First key
	while (LUA->Next(1)) {
		CPhysConvex *pConvex = Get_PhysConvex(state, -1);
		convexes.AddToTail(pConvex);

		// Pop the value
		LUA->Pop();
	}

	CPhysCollide *pCollide = g_pPhysCollision->ConvertConvexToCollide(convexes.Base(), convexes.Count());
	Push_PhysCollide(state, pCollide);
	return 1;
}

int lPhysCollisionConvexFree(lua_State *state) {
	CPhysConvex *pConvex = Get_PhysConvex(state, 1);
	g_pPhysCollision->ConvexFree(pConvex);

	LUA->PushNil();
	LUA->SetMetaTable(1);
	return 0;
}

//
// Name: physcollision.CylinderToConvex
// Desc: Converts AABB to a cylinder. Top/bottom sides are flat.
// Arg1: Vector|mins|
// Arg2: Vector|maxs|
// Ret1: CPhysConvex|convex|The convex mesh.
//
int lPhysCollisionCylinderToConvex(lua_State *state) {
	Vector *pMins = Get_Vector(state, 1);
	Vector *pMaxs = Get_Vector(state, 2);

	CPhysConvex *pConvex = g_pPhysCollision->CylinderToConvex(*pMins, *pMaxs);
	Push_PhysConvex(state, pConvex);
	return 1;
}

int lPhysCollisionSphereToConvex(lua_State *state) {
	float radius = LUA->GetNumber(1);

	CPhysConvex *pConvex = g_pPhysCollision->SphereToConvex(radius);
	Push_PhysConvex(state, pConvex);
	return 1;
}

int lPhysCollisionConeToConvex(lua_State *state) {
	float radius = LUA->GetNumber(1);
	float height = LUA->GetNumber(2);

	CPhysConvex *pConvex = g_pPhysCollision->ConeToConvex(radius, height);
	Push_PhysConvex(state, pConvex);
	return 1;
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
			LUA->PushCFunction(lPhysCollisionDestroyCollide);	LUA->SetField(-2, "DestroyCollide");

			LUA->PushCFunction(lPhysCollisionConvertConvexesToCollide); LUA->SetField(-2, "ConvertConvexesToCollide");
			LUA->PushCFunction(lPhysCollisionConvexFree);		LUA->SetField(-2, "ConvexFree");
			LUA->PushCFunction(lPhysCollisionCylinderToConvex);	LUA->SetField(-2, "CylinderToConvex");
			LUA->PushCFunction(lPhysCollisionSphereToConvex);	LUA->SetField(-2, "SphereToConvex");
			LUA->PushCFunction(lPhysCollisionConeToConvex);		LUA->SetField(-2, "ConeToConvex");
		LUA->SetField(-2, "physcollision");
	LUA->Pop();

	return 0;
}