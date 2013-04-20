#ifndef MISCFUNCS_H
#define MISCFUNCS_H

class IPhysicsObject1;
class CPhysCollide;
class Vector;
struct lua_State;

IPhysicsObject1 *Get_PhysObj(lua_State *state, int stackPos);
Vector *Get_Vector(lua_State *state, int stackPos);
CPhysCollide *Get_PhysCollide(lua_State *state, int stackPos);

void Push_Vector(lua_State *state, const Vector &vec);
void Push_PhysCollide(lua_State *state, const CPhysCollide *collide);

#endif // MISCFUNCS_H