#include "MiscFuncs.h"

#include "../include/vphysics_interfaceV32.h"
#include <GarrysMod/Lua/Interface.h>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

// Prints argument error (type expected, type received)
inline void PrintArgError(lua_State *state, int stackPos, const char *argType) {
	static char str[1024];
	sprintf_s(str, "expected %s, got %s", argType, LUA->GetTypeName(LUA->GetType(stackPos)));
	LUA->ArgError(stackPos, str);
}

IPhysicsEnvironment32 *Get_PhysEnv(lua_State *state, int stackPos) {
	LUA->CheckType(stackPos, CustomTypes::TYPE_PHYSENV);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	return (IPhysicsEnvironment32 *)ud->data;
}

void Push_PhysEnv(lua_State *state, IPhysicsEnvironment32 *pEnv) {
	UserData *ud = (UserData *)LUA->NewUserdata(sizeof(UserData));
	ud->type = CustomTypes::TYPE_PHYSENV;
	ud->data = pEnv;

	LUA->CreateMetaTableType("PhysEnv", CustomTypes::TYPE_PHYSENV);
	LUA->SetMetaTable(-2);
}

IPhysicsObject32 *Get_PhysObj(lua_State *state, int stackPos) {
	LUA->CheckType(stackPos, Type::PHYSOBJ);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	return (IPhysicsObject32 *)ud->data;
}

CPhysCollide *Get_PhysCollide(lua_State *state, int stackPos) {
	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	if (ud->type != CustomTypes::TYPE_PHYSCOLLIDE) {
		PrintArgError(state, stackPos, "PhysCollide");
		return NULL;
	}

	return (CPhysCollide *)ud->data;
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

QAngle *Get_Angle(lua_State *state, int stackPos) {
	LUA->CheckType(stackPos, Type::ANGLE);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	return (QAngle *)ud->data;
}

IPhysicsSoftBody *Get_SoftBody(lua_State *state, int stackPos) {
	LUA->CheckType(stackPos, CustomTypes::TYPE_PHYSSOFTBODY);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	return (IPhysicsSoftBody *)ud->data;
}

void Push_SoftBody(lua_State *state, IPhysicsSoftBody *softBody) {
	UserData *ud = (UserData *)LUA->NewUserdata(sizeof(UserData));
	ud->type = CustomTypes::TYPE_PHYSSOFTBODY;
	ud->data = (void *)softBody;

	LUA->CreateMetaTableType("PhysSoftBody", CustomTypes::TYPE_PHYSSOFTBODY);
	LUA->SetMetaTable(-2);
}

void Push_PhysCollide(lua_State *state, const CPhysCollide *collide) {
	UserData *ud = (UserData *)LUA->NewUserdata(sizeof(UserData));
	ud->data = (void *)collide;
	ud->type = CustomTypes::TYPE_PHYSCOLLIDE;
}

CPhysConvex *Get_PhysConvex(lua_State *state, int stackPos) {
	LUA->CheckType(stackPos, CustomTypes::TYPE_PHYSCONVEX);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	return (CPhysConvex *)ud->data;
}

void Push_PhysConvex(lua_State *state, const CPhysConvex *convex) {
	UserData *ud = (UserData *)LUA->NewUserdata(sizeof(UserData));
	ud->type = CustomTypes::TYPE_PHYSCONVEX;
	ud->data = (void *)convex;

	LUA->CreateMetaTableType("PhysConvex", CustomTypes::TYPE_PHYSCONVEX);
	LUA->SetMetaTable(-2);
}