#ifndef MISCFUNCS_H
#define MISCFUNCS_H

class IPhysicsObject1;
class Vector;
struct lua_State;

IPhysicsObject1 *Get_PhysObj(lua_State *state, int stackPos);
Vector *Get_Vector(lua_State *state, int stackPos);

void Push_Vector(lua_State *state, const Vector &vec);

#endif // MISCFUNCS_H