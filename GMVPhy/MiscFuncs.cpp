#include "MiscFuncs.h"

#include "../include/vphysics_interfaceV32.h"
#include <GarrysMod/Lua/Interface.h>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

namespace CustomTypes {
	enum {
		TYPE_PHYSCOLLIDE = Type::COUNT + 1
	};
}

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

CPhysCollide *Get_PhysCollide(lua_State *state, int stackPos) {
	//LUA->CheckType(stackPos, Type::USERDATA);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	if (ud->type != CustomTypes::TYPE_PHYSCOLLIDE) {
		LUA->ArgError(stackPos, "expected a PhysCollide");
		return NULL;
	}

	return (CPhysCollide *)ud->data;
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

void Push_PhysCollide(lua_State *state, const CPhysCollide *collide) {
	UserData *ud = (UserData *)LUA->NewUserdata(sizeof(UserData));
	ud->data = (void *)collide;
	ud->type = CustomTypes::TYPE_PHYSCOLLIDE;
}