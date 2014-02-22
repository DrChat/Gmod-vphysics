#include "PhysSoftBody.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include <tier1/tier1.h>
#include "../include/vphysics_interfaceV32.h"
#include "../include/vphysics/softbodyV32.h"

#include <cmodel.h> // Ray_t

#include "MiscFuncs.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// TODO: Soft bodies should be implemented as an entity later on!
//
// Name: softbody.CreateFromVertices
// Desc: Creates a new soft body from vertex array
// Arg1: PhysEnv|env|The physics environment in which to create the soft body
// Arg2: Table|verts|The vertices
// Ret1: PhysSoftBody|softbody|The new soft body
//
int lCreateSoftBodyFromVertices(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);
	LUA->CheckType(2, GarrysMod::Lua::Type::TABLE);

	CUtlVector<Vector> vertices;

	// Loop through table
	LUA->PushNil(); // Key (will be popped by lua_next automatically)
	while (LUA->Next(2)) {
		// Pos -1 is now the value
		Vector *pVec = Get_Vector(state, -1);
		vertices.AddToTail(*pVec);

		LUA->Pop(); // Pop the value, keep the key for traversal
	}

	IPhysicsSoftBody *pSoftBody = pEnv->CreateSoftBodyFromVertices(&vertices[0], vertices.Count(), NULL);
	if (!pSoftBody)
		LUA->ThrowError("Failed to create soft body (environment returned NULL)");

	Push_SoftBody(state, pSoftBody);

	return 1;
}

//
// Name: softbody.CreateRope
// Desc: Creates a new soft body rope
// Arg1: PhysEnv|env|The physics environment to create the soft body in
// Arg2: Vector|start|Start position
// Arg3: Vector|length|Rope length
// Arg4: int|resolution|Soft body resolution (# of nodes)
// Ret1: PhysSoftBody|softbody|The rope soft body
//
int lCreateSoftBodyRope(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);
	Vector *pos = Get_Vector(state, 2);
	Vector *length = Get_Vector(state, 3);
	int res = LUA->GetNumber(4);

	IPhysicsSoftBody *pSoftBody = pEnv->CreateSoftBodyRope(*pos, *length, res, NULL);
	if (!pSoftBody)
		LUA->ThrowError("Failed to create soft body (environment returned NULL)");

	Push_SoftBody(state, pSoftBody);
	return 1;
}

int lCreateSoftBodyPatch(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);
	LUA->CheckType(2, GarrysMod::Lua::Type::TABLE);

	Vector corners[4];

	LUA->PushNil();
	int i = 0;
	while (LUA->Next(2) != 0) {
		Vector *vec = Get_Vector(state, -1);
		corners[i] = *vec;

		i++;
		if (i > 4) LUA->ThrowError("More than 4 vertices given!"); // Protect against user error

		LUA->Pop(); // Pop the value on the top of the stack
	}

	int resx = LUA->GetNumber(3);
	int resy = LUA->GetNumber(4);

	IPhysicsSoftBody *pSoftBody = pEnv->CreateSoftBodyPatch(corners, resx, resy, NULL);
	if (!pSoftBody)
		LUA->ThrowError("Failed to create soft body (environment returned NULL)");

	Push_SoftBody(state, pSoftBody);
	return 1;
}

int lDestroySoftBody(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	pSoftBody->GetPhysicsEnvironment()->DestroySoftBody(pSoftBody);

	// Set a nil metatable so the user can't call any more functions on this
	LUA->PushNil();
	LUA->SetMetaTable(1);

	return 0;
}

/****************************
* PhysSoftBody CLASS
****************************/
int lPhysSoftBodySetTotalMass(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	float mass = LUA->GetNumber(2);

	pSoftBody->SetTotalMass(mass);

	return 0;
}

int lPhysSoftBodyAppendAnchor(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	IPhysicsObject32 *pObj = Get_PhysObj(state, 2);
	int node = LUA->GetNumber(3) - 1; // - 1 because lua arrays start at 1
	if (node < 0 || node >= pSoftBody->GetNodeCount())
		LUA->ThrowError("Soft body node is out of bounds");

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

int lPhysSoftBodySetNode(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	int nodeId = LUA->GetNumber(2);

	if (nodeId < 0 || nodeId >= pSoftBody->GetNodeCount())
		LUA->ThrowError("Out of bounds node ID");

	return 0;
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

int lPhysSoftBodyGetLinks(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);

	if (pSoftBody->GetLinkCount()) {
		LUA->CreateTable(); // table returned
		for (int i = 0; i < pSoftBody->GetLinkCount(); i++) {
			softbodylink_t link = pSoftBody->GetLink(i);
			LUA->PushNumber(i);
			LUA->CreateTable(); // link

			// nodes
			LUA->CreateTable();
			for (int i = 0; i < 2; i++) {
				LUA->PushNumber(i); // id

				// node
				LUA->CreateTable();
				Push_Vector(state, link.nodes[i].pos);
				LUA->SetField(-2, "pos");
				Push_Vector(state, link.nodes[i].vel);
				LUA->SetField(-2, "vel");
				LUA->PushNumber(link.nodes[i].invMass);
				LUA->SetField(-2, "invMass");
				LUA->PushNumber(link.nodeIndexes[i] + 1); // + 1 because lua arrays start at 1
				LUA->SetField(-2, "index");

				LUA->SetTable(-3);
			}
			LUA->SetField(-2, "nodes");

			LUA->SetTable(-3); // set link to parent table
		}
	} else {
		LUA->PushBool(false);
	}

	return 1;
}

int lPhysSoftBodyGetFaces(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);

	if (pSoftBody->GetFaceCount()) {
		LUA->CreateTable();
		for (int i = 0; i < pSoftBody->GetFaceCount(); i++) {
			softbodyface_t face = pSoftBody->GetFace(i);
			LUA->PushNumber(i);
			LUA->CreateTable(); // face

			// Nodes
			LUA->CreateTable();
			for (int i = 0; i < 3; i++) {
				softbodynode_t &node = face.nodes[i];
				LUA->PushNumber(i); // Key

				LUA->CreateTable(); // Value
				Push_Vector(state, node.pos);
				LUA->SetField(-2, "pos");
				Push_Vector(state, node.vel);
				LUA->SetField(-2, "vel");
				LUA->PushNumber(node.invMass);
				LUA->SetField(-2, "invMass");
				LUA->PushNumber(face.nodeIndexes[i] + 1); // +1 because lua arrays start at 1
				LUA->SetField(-2, "index");

				LUA->SetTable(-3); // t(-3)[Key(-2)] = Value(-1)
			}
			LUA->SetField(-2, "nodes");

			Push_Vector(state, face.normal);
			LUA->SetField(-2, "normal");

			LUA->SetTable(-3);
		}
	} else {
		LUA->PushBool(false);
	}

	return 1;
}

//
// Name: PhysSoftBody:GetAABB
// Desc: Get the AABB of a soft body
// Arg1: PhysSoftBody|softbody|
// Ret1: Vector|mins|
// Ret2: Vector|maxs|
//
int lPhysSoftBodyGetAABB(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);

	Vector mins, maxs;
	pSoftBody->GetAABB(&mins, &maxs);

	Push_Vector(state, mins);
	Push_Vector(state, maxs);

	return 2;
}

//
// Name: PhysSoftBody:RayTest
// Desc: Test a ray
// Arg1: PhysSoftBody|softbody|
// Arg2: Table|ray|{start=Vector(), end=Vector()}
// Ret1: Table|result|Result in the style of Structures/TraceResult
//
int lPhysSoftBodyRayTest(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	LUA->CheckType(2, GarrysMod::Lua::Type::TABLE);

	Ray_t ray;
	Vector start, end;
	LUA->PushString("start");
	LUA->GetTable(2);
	start = *Get_Vector(state, -1);
	LUA->Pop();

	LUA->PushString("end");
	LUA->GetTable(2);
	end = *Get_Vector(state, -1);
	LUA->Pop();

	ray.Init(start, end);

	trace_t tr;
	pSoftBody->RayTest(ray, &tr);

	LUA->CreateTable();
	LUA->PushNumber(tr.fraction);
	LUA->SetField(-2, "Fraction");
	LUA->PushBool(tr.DidHit());
	LUA->SetField(-2, "Hit");
	Push_Vector(state, tr.plane.normal);
	LUA->SetField(-2, "HitNormal");
	Push_Vector(state, tr.startpos);
	LUA->SetField(-2, "StartPos");
	Push_Vector(state, tr.endpos);
	LUA->SetField(-2, "EndPos");
	Push_Vector(state, ray.m_Delta.Normalized());
	LUA->SetField(-2, "Normal");

	return 1;
}

//
// Name: PhysSoftBody:Translate
// Desc: Translates a softbody
// Arg1: PhysSoftBody|softbody|
// Arg2: Vector|offset|Offset to move the softbody
// Ret1: 
//
int lPhysSoftBodyTranslate(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	Vector *pos = Get_Vector(state, 2);

	pSoftBody->Transform(pos, NULL);
	return 0;
}

//
// Name: PhysSoftBody:Rotate
// Desc: Rotates a softbody
// Arg1: PhysSoftBody|softbody|
// Arg2: Angle|ang|Angle offset to rotate the softbody
// Ret1: 
//
int lPhysSoftBodyRotate(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	QAngle *ang = Get_Angle(state, 2);

	pSoftBody->Transform(NULL, ang);
	return 0;
}

//
// Name: PhysSoftBody:Scale
// Desc: Scales a softbody
// Arg1: PhysSoftBody|softbody|
// Arg2: Vector|scl|Scale
// Ret1: 
//
int lPhysSoftBodyScale(lua_State *state) {
	IPhysicsSoftBody *pSoftBody = Get_SoftBody(state, 1);
	Vector *scl = Get_Vector(state, 2);

	pSoftBody->Scale(*scl);
	return 0;
}

int Init_PhysSoftBody(lua_State *state) {
	LUA->CreateMetaTableType("PhysSoftBody", CustomTypes::TYPE_PHYSSOFTBODY);
		LUA->Push(-1); LUA->SetField(-2, "__index"); // Allow function and field lookups on this table
		LUA->PushCFunction(lPhysSoftBodySetTotalMass); LUA->SetField(-2, "SetTotalMass");
		LUA->PushCFunction(lPhysSoftBodyAppendAnchor); LUA->SetField(-2, "AppendAnchor");
		LUA->PushCFunction(lPhysSoftBodyGetNodeCount); LUA->SetField(-2, "GetNodeCount");
		LUA->PushCFunction(lPhysSoftBodyGetFaces); LUA->SetField(-2, "GetFaces");
		LUA->PushCFunction(lPhysSoftBodyGetNodes); LUA->SetField(-2, "GetNodes");
		LUA->PushCFunction(lPhysSoftBodyGetLinks); LUA->SetField(-2, "GetLinks");

		LUA->PushCFunction(lPhysSoftBodyGetAABB); LUA->SetField(-2, "GetAABB");
		LUA->PushCFunction(lPhysSoftBodyRayTest); LUA->SetField(-2, "RayTest");

		LUA->PushCFunction(lPhysSoftBodyTranslate); LUA->SetField(-2, "Translate");
		LUA->PushCFunction(lPhysSoftBodyRotate); LUA->SetField(-2, "Rotate");
		LUA->PushCFunction(lPhysSoftBodyScale); LUA->SetField(-2, "Scale");
	LUA->Pop();

	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_GLOB);
		LUA->CreateTable();
			LUA->PushCFunction(lCreateSoftBodyFromVertices); LUA->SetField(-2, "CreateFromVertices");
			LUA->PushCFunction(lCreateSoftBodyRope); LUA->SetField(-2, "CreateRope");
			LUA->PushCFunction(lCreateSoftBodyPatch); LUA->SetField(-2, "CreatePatch");
			LUA->PushCFunction(lDestroySoftBody); LUA->SetField(-2, "Destroy");
		LUA->SetField(-2, "softbody");
	LUA->Pop();


	return 0;
}