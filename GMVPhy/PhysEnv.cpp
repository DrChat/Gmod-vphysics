#include "PhysEnv.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"

#include "MiscFuncs.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

extern IPhysics32 *g_pPhysics;

// Special metatable functions
int lPhysEnv__tostring(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);

	char str[1024];
	sprintf_s(str, "Physics Environment - %d active objects", pEnv->GetActiveObjectCount());
	LUA->PushString(str);

	return 1;
}

//
// Name: PhysEnv:SetGravity
// Desc: Sets the global environment gravity
// Arg1: Vector|grav|Gravity vector
// Ret1: 
//
int lPhysEnvSetGravity(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);
	Vector *pGravity = Get_Vector(state, 2);

	pEnv->SetGravity(*pGravity);

	return 0;
}

//
// Name: PhysEnv:GetGravity
// Desc: Get the global environment gravity
// Arg1:
// Ret1: Vector|grav|Gravity vector
//
int lPhysEnvGetGravity(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);

	Vector gravity;
	pEnv->GetGravity(&gravity);
	Push_Vector(state, gravity);

	return 1;
}

//
// Name: PhysEnv:Simulate
// Desc: Advances the simulation by deltaTime seconds
// Arg1: Number|deltaTime|Number of seconds to advance
// Ret1: 
//
int lPhysEnvSimulate(lua_State *state) {
	IPhysicsEnvironment32 *pPhysEnv = Get_PhysEnv(state, 1);
	float deltaTime = LUA->GetNumber(2);

	pPhysEnv->Simulate(deltaTime);

	return 0;
}

//
// Name: PhysEnv:TransferObject
// Desc: Transfers an object to another environment
// Arg1: PhysEnv|self|self
// Arg2: PhysObj|object|object to transfer
// Arg3: PhysEnv|dest|destination environment
// Ret1: bool|success|Transfer success
//
int lPhysEnvTransferObject(lua_State *state) {
	IPhysicsEnvironment32 *pPhysEnv = Get_PhysEnv(state, 1);
	IPhysicsObject32 *pPhysObj = Get_PhysObj(state, 2);
	IPhysicsEnvironment32 *pDestEnv = Get_PhysEnv(state, 3);

	LUA->PushBool(pPhysEnv->TransferObject(pPhysObj, pDestEnv));
	return 1;
}

//
// Name: PhysEnv:IsCollisionModelUsed
// Desc: Is the CPhysCollide being used by any objects?
// Arg1: PhysEnv|self|
// Arg2: PhysCollide|collide|collision model
// Ret1: bool|used|Used?
//
int lPhysEnvIsCollisionModelUsed(lua_State *state) {
	IPhysicsEnvironment32 *pPhysEnv = Get_PhysEnv(state, 1);
	CPhysCollide *pCollide = Get_PhysCollide(state, 2);

	LUA->PushBool(pPhysEnv->IsCollisionModelUsed(pCollide));
	return 1;
}

//
// Name: physenv.CreateNew
// Desc: Creates a new physics environment
// Arg1:
// Ret1: PhysEnv|env|new environment
//
int lPhysEnvCreateNew(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = (IPhysicsEnvironment32 *)g_pPhysics->CreateEnvironment();

	Push_PhysEnv(state, pEnv);

	return 1;
}

//
// Name: physenv.Destroy
// Desc: Destroys an environment. Throws an error if the environment contains any physics objects (or the game will crash in server)
// DO NOT USE THIS TO DELETE THE GAME'S INTERNAL ENVIRONMENTS!!!!
// Arg1: PhysEnv|environment|The environment to destroy
// Ret1:
//
int lPhysEnvDestroy(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);
	// Crashes in server.dll without this check.
	if (pEnv->GetObjectCount() > 0)
		LUA->ThrowError("Cannot remove an environment that contains objects!");

	g_pPhysics->DestroyEnvironment(pEnv);

	// Invalidate the metatable
	LUA->PushNil();
	LUA->SetMetaTable(1);

	return 0;
}

//
// Name: physenv.GetActiveEnvironmentCount
// Desc: Returns the number of active environments
// Arg1:
// Ret1: number|numEnvironments|number of active environments
//
int lPhysEnvGetActiveEnvironmentCount(lua_State *state) {
	LUA->PushNumber(g_pPhysics->GetActiveEnvironmentCount());
	return 1;
}

//
// Name: physenv.GetActiveEnvironmentByIndex
// Desc: Gets a physics environment by a specified index
// Arg1:
// Ret1: PhysEnv|env|Environment
//
int lPhysEnvGetActiveEnvironmentByIndex(lua_State *state) {
	int idx = LUA->GetNumber(1);

	if (idx < 0 || idx >= g_pPhysics->GetActiveEnvironmentCount())
		LUA->ThrowError("Environment index out of bounds");

	IPhysicsEnvironment32 *pEnv = (IPhysicsEnvironment32 *)g_pPhysics->GetActiveEnvironmentByIndex(idx);
	Push_PhysEnv(state, pEnv);

	return 1;
}

int Init_PhysEnv(lua_State *state) {
	// Setup the physics environment metatable
	LUA->CreateMetaTableType("PhysEnv", CustomTypes::TYPE_PHYSENV);
		LUA->Push(-1); LUA->SetField(-2, "__index");
		LUA->PushCFunction(lPhysEnv__tostring); LUA->SetField(-2, "__tostring");
		LUA->PushCFunction(lPhysEnvSetGravity); LUA->SetField(-2, "SetGravity");
		LUA->PushCFunction(lPhysEnvGetGravity); LUA->SetField(-2, "GetGravity");
		LUA->PushCFunction(lPhysEnvSimulate); LUA->SetField(-2, "Simulate");
		LUA->PushCFunction(lPhysEnvTransferObject); LUA->SetField(-2, "TransferObject");
		LUA->PushCFunction(lPhysEnvIsCollisionModelUsed); LUA->SetField(-2, "IsCollisionModelUsed");
	LUA->Pop(1);

	LUA->PushSpecial(SPECIAL_GLOB);
		LUA->GetField(-1, "physenv");
			LUA->PushCFunction(lPhysEnvCreateNew); LUA->SetField(-2, "CreateNew");
			LUA->PushCFunction(lPhysEnvDestroy); LUA->SetField(-2, "Destroy");
			LUA->PushCFunction(lPhysEnvGetActiveEnvironmentCount); LUA->SetField(-2, "GetActiveEnvironmentCount");
			LUA->PushCFunction(lPhysEnvGetActiveEnvironmentByIndex); LUA->SetField(-2, "GetActiveEnvironmentByIndex");
	LUA->Pop(2);

	return 0;
}