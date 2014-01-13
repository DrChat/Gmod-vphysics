#ifndef MISCFUNCS_H
#define MISCFUNCS_H

class IPhysicsEnvironment32;
class IPhysicsObject32;
class CPhysCollide;
class Vector;
struct lua_State;

#include <GarrysMod/Lua/Interface.h>

namespace CustomTypes {
	enum {
		TYPE_PHYSCOLLIDE = GarrysMod::Lua::Type::COUNT + 1,
		TYPE_PHYSENV,

		COUNT,
	};
}

IPhysicsEnvironment32 *Get_PhysEnv(lua_State *state, int stackPos);
void Push_PhysEnv(lua_State *state, IPhysicsEnvironment32 *pEnv);

IPhysicsObject32 *Get_PhysObj(lua_State *state, int stackPos);

Vector *Get_Vector(lua_State *state, int stackPos);
void Push_Vector(lua_State *state, const Vector &vec);

CPhysCollide *Get_PhysCollide(lua_State *state, int stackPos);
void Push_PhysCollide(lua_State *state, const CPhysCollide *collide);

#endif // MISCFUNCS_H