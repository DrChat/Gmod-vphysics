#include "PhysSoftBody.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include <tier1/tier1.h>
#include "../include/vphysics_interfaceV32.h"
#include "../include/vphysics/softbodyV32.h"

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

int lCreateSoftBodyRope(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);
	Vector *pos = Get_Vector(state, 2);
	Vector *length = Get_Vector(state, 3);

	IPhysicsSoftBody *pSoftBody = pEnv->CreateSoftBodyRope(*pos, *length, NULL);
	if (!pSoftBody)
		LUA->ThrowError("Failed to create soft body (environment returned NULL)");

	Push_SoftBody(state, pSoftBody);
	return 1;
}

int lPhysSoftBodySetTotalMass(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	float mass = LUA->GetNumber(2);

	pSoftBody->SetTotalMass(mass);

	return 0;
}

int lPhysSoftBodyAppendAnchor(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	IPhysicsObject32 *pObj = Get_PhysObj(state, 2);
	int node = LUA->GetNumber(3);

	pSoftBody->Anchor(node, pObj);
	return 0;
}

int lPhysSoftBodyGetNodeCount(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	LUA->PushNumber(pSoftBody->GetNodeCount());

	return 1;
}

int lPhysSoftBodyGetFaceCount(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	LUA->PushNumber(pSoftBody->GetFaceCount());

	return 1;
}

int lPhysSoftBodyGetFaces(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);

	if (pSoftBody->GetFaceCount()) {
		LUA->CreateTable();
		for (int i = 0; i < pSoftBody->GetFaceCount(); i++) {
			softbodyface_t face = pSoftBody->GetFace(i);

			// Nodes
			LUA->CreateTable();
			for (int i = 0; i < 3; i++) {
				softbodynode_t node = face.nodes[i];

				LUA->PushNumber(i); // Key
				LUA->CreateTable(); // Value
				Push_Vector(state, node.pos);
				LUA->SetField(-2, "pos");
				Push_Vector(state, node.vel);
				LUA->SetField(-2, "vel");
				LUA->PushNumber(node.invMass);
				LUA->SetField(-2, "invMass");

				LUA->SetTable(-3); // t(-3)[Key(-2)] = Value(-1)
			}
			LUA->SetField(-2, "nodes");

			Push_Vector(state, face.normal);
			LUA->SetField(-2, "normal");
		}
	} else {
		LUA->PushBool(false);
	}

	return 1;
}

int lPhysSoftBodyGetNodes(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);

	if (pSoftBody->GetNodeCount()) {
		LUA->CreateTable();
		for (int i = 0; i < pSoftBody->GetNodeCount(); i++) {
			softbodynode_t node = pSoftBody->GetNode(i);

			LUA->PushNumber(i);
			LUA->CreateTable();
			Push_Vector(state, node.pos);
			LUA->SetField(-2, "pos");
			Push_Vector(state, node.vel);
			LUA->SetField(-2, "vel");
			LUA->PushNumber(node.invMass);
			LUA->SetField(-2, "invMass");

			LUA->SetTable(-3); // t(-3)[Key(-2)] = Value(-1)
		}
	} else {
		LUA->PushBool(false);
	}

	return 1;
}

int Init_PhysSoftBody(lua_State *state) {
	LUA->CreateMetaTableType("PhysSoftBody", CustomTypes::TYPE_PHYSSOFTBODY);
		LUA->Push(-1); LUA->SetField(-2, "__index"); // Allow function and field lookups on this table
		LUA->PushCFunction(lPhysSoftBodySetTotalMass); LUA->SetField(-2, "SetTotalMass");
		LUA->PushCFunction(lPhysSoftBodyAppendAnchor); LUA->SetField(-2, "AppendAnchor");
		LUA->PushCFunction(lPhysSoftBodyGetNodeCount); LUA->SetField(-2, "GetNodeCount");
		LUA->PushCFunction(lPhysSoftBodyGetFaces); LUA->SetField(-2, "GetFaces");
		LUA->PushCFunction(lPhysSoftBodyGetNodes); LUA->SetField(-2, "GetNodes");
	LUA->Pop();

	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_GLOB);
		LUA->CreateTable();
			LUA->PushCFunction(lCreateSoftBodyRope); LUA->SetField(-2, "CreateRope");
		LUA->SetField(-2, "softbody");
	LUA->Pop();


	return 0;
}