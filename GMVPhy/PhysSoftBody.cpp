#include "PhysSoftBody.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include <tier1/tier1.h>
#include "../include/vphysics_interfaceV32.h"

#include "MiscFuncs.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// TODO: Soft bodies should be implemented as an entity later on!
int lCreateSoftBodyFromVerts(lua_State *state) {
	LUA->CheckType(-1, GarrysMod::Lua::Type::TABLE);

	LUA->PushNil(); // Key

	CUtlVector<Vector> vertices;

	// Loop through table
	while (LUA->Next(-3)) {
		// Pos -1 is now the value
		Vector *pVec = Get_Vector(state, -1);
		vertices.AddToTail(*pVec);

		LUA->Pop(); // Pop the value, keep the key for traversal
	}

	LUA->Pop(); // Pop key
	return 0; // TODO: Return soft body object
}

int Init_PhysSoftBody(lua_State *state) {
	LUA->CreateMetaTableType("PhysSoftBody", CustomTypes::TYPE_PHYSSOFTBODY);
		LUA->Push(-1); LUA->SetField(-2, "__index"); // Allow function and field lookups on this table
	LUA->Pop();

	return 0;
}