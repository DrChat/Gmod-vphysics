#ifndef MISCFUNCS_H
#define MISCFUNCS_H

class IPhysicsEnvironment32;
class IPhysicsObject32;
class IPhysicsSoftBody;
class CPhysCollide;
class CPhysConvex;
class Vector;
class QAngle;
struct lua_State;

#include <GarrysMod/Lua/Interface.h>

// These should probably be in a stdafx.h or something
namespace CustomTypes {
	enum {
		TYPE_PHYSCOLLIDE = GarrysMod::Lua::Type::COUNT + 1,
		TYPE_PHYSCONVEX,

		TYPE_PHYSENV,
		TYPE_PHYSSOFTBODY,
		TYPE_PHYSUSERCONSTRAINT,

		COUNT,
	};
}

IPhysicsEnvironment32 *Get_PhysEnv(lua_State *state, int stackPos);
void Push_PhysEnv(lua_State *state, IPhysicsEnvironment32 *pEnv);

IPhysicsObject32 *Get_PhysObj(lua_State *state, int stackPos);

Vector *Get_Vector(lua_State *state, int stackPos);
void Push_Vector(lua_State *state, const Vector &vec);

QAngle *Get_Angle(lua_State *state, int stackPos);

IPhysicsSoftBody *Get_SoftBody(lua_State *state, int stackPos);
void Push_SoftBody(lua_State *state, IPhysicsSoftBody *softBody);

CPhysCollide *Get_PhysCollide(lua_State *state, int stackPos);
void Push_PhysCollide(lua_State *state, const CPhysCollide *collide);

CPhysConvex *Get_PhysConvex(lua_State *state, int stackPos);
void Push_PhysConvex(lua_State *state, const CPhysConvex *convex);

#endif // MISCFUNCS_H