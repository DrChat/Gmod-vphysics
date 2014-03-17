#include "PhysConstraint.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"
#include "../include/vphysics/constraintsV32.h"

#include "MiscFuncs.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

using namespace GarrysMod::Lua;

class CLuaUserConstraint : public IPhysicsUserConstraint {
	public:
		CLuaUserConstraint(ILuaBase *pLua, lua_State *pState, int ref) {
			m_pLua = pLua;
			m_pLuaState = pState;
			m_tableReference = ref;
		}

		~CLuaUserConstraint() {
			m_pLua->ReferenceFree(m_tableReference);
		}

		void ConstraintDestroyed(IPhysicsConstraint *pConstraint) {
			// That's all folks!
			delete this;
		}

		// Purpose: Get basic constraint info (# rows) for initialization
		void GetConstraintInfo(IPhysicsObject *pObjA, IPhysicsObject *pObjB, physconstraintinfo_t &info) {
			m_pLua->ReferencePush(m_tableReference);
			m_pLua->GetField(-1, "GetConstraintInfo"); // No checking (assumed already done on creation)
			
			m_pLua->Call(0, 1);
			if (!m_pLua->IsType(-1, Type::TABLE)) {
				m_pLua->ThrowError("GetConstraintInfo did not return a table!");
			}

			m_pLua->GetField(-1, "numConstraintRows");
			info.numConstraintRows = m_pLua->GetNumber(-1);
			m_pLua->Pop();

			m_pLua->GetField(-1, "nub");
			info.nub = m_pLua->GetNumber(-1);
			m_pLua->Pop();

			int top = m_pLua->Top();

			// Pop the returned table and the referenced table
			m_pLua->Pop(2);
		}

		void GetConstraintSolveInfo(IPhysicsObject *pObjA, IPhysicsObject *pObjB, physconstraintsolveinfo_t *info, int numRows, float fps, float erp) {
			m_pLua->ReferencePush(m_tableReference);
			m_pLua->GetField(-1, "GetConstraintSolveInfo");

			Push_PhysObj(m_pLuaState, pObjA);
			Push_PhysObj(m_pLuaState, pObjB);
			m_pLua->PushNumber(numRows);
			m_pLua->PushNumber(fps);
			m_pLua->PushNumber(erp);
			m_pLua->Call(5, 1);

			// User should return a table with numRows inner tables
			// The inner tables are the actual constraint rows
			m_pLua->CheckType(-1, Type::TABLE);

			int top = m_pLua->Top();

			m_pLua->PushNil(); // Key
			int next = m_pLua->Next(-2);
			for (int i = 0; i < numRows; i++) {
				if (!next)
					m_pLua->ThrowError("Returned table does not contain enough constraint rows!");
				m_pLua->CheckType(-1, Type::TABLE); // Check the value to make sure it's a table

				physconstraintsolveinfo_t &curInfo = info[i];
				m_pLua->GetField(-1, "J1linearAxis");
				if (!m_pLua->IsType(-1, Type::NIL))
					curInfo.J1linearAxis = *Get_Vector(m_pLuaState, -1);
				m_pLua->Pop();
				
				m_pLua->GetField(-1, "J2linearAxis");
				if (!m_pLua->IsType(-1, Type::NIL))
					curInfo.J2linearAxis = *Get_Vector(m_pLuaState, -1);
				m_pLua->Pop();

				m_pLua->GetField(-1, "J1angularAxis");
				if (!m_pLua->IsType(-1, Type::NIL))
					curInfo.J1angularAxis = *Get_Vector(m_pLuaState, -1);
				m_pLua->Pop();

				m_pLua->GetField(-1, "J2angularAxis");
				if (!m_pLua->IsType(-1, Type::NIL))
					curInfo.J2angularAxis = *Get_Vector(m_pLuaState, -1);
				m_pLua->Pop();

				// Make sure the jacobian sum is nonzero
				float sum = curInfo.J1linearAxis.LengthSqr() + curInfo.J1angularAxis.LengthSqr()
					+ curInfo.J2linearAxis.LengthSqr() + curInfo.J2angularAxis.LengthSqr();
				if (sum <= 0.f) {
					Warning("User constraint row %d has jacobian sum of zero!\n", i);
				}

				m_pLua->GetField(-1, "constraintError");
				if (!m_pLua->IsType(-1, Type::NIL))
					curInfo.constraintError = m_pLua->GetNumber();
				m_pLua->Pop();

				// Optional
				m_pLua->GetField(-1, "cfm");
				if (!m_pLua->IsType(-1, Type::NIL))
					curInfo.cfm = m_pLua->GetNumber();
				m_pLua->Pop();

				// Optional
				m_pLua->GetField(-1, "lowerLimit");
				if (!m_pLua->IsType(-1, Type::NIL))
					curInfo.lowerLimit = m_pLua->GetNumber();
				m_pLua->Pop();

				// Optional
				m_pLua->GetField(-1, "upperLimit");
				if (!m_pLua->IsType(-1, Type::NIL))
					curInfo.upperLimit = m_pLua->GetNumber();
				m_pLua->Pop();

				top = m_pLua->Top();

				// Pop the value
				m_pLua->Pop();
				int next = m_pLua->Next(-2);

				top = m_pLua->Top();
			}
			// Pop the key
			m_pLua->Pop();
			top = m_pLua->Top();

			// Pop the ref and the return table
			m_pLua->Pop();

			top = m_pLua->Top();
		}

	private:
		lua_State *	m_pLuaState;
		ILuaBase *	m_pLua;
		int			m_tableReference;
};

void Push_UserConstraint(lua_State *state, CLuaUserConstraint *pConstraint) {
	UserData *ud = (UserData *)LUA->NewUserdata(sizeof(UserData));
	ud->type = CustomTypes::TYPE_PHYSUSERCONSTRAINT;
	ud->data = pConstraint;

	LUA->CreateMetaTableType("PhysUserConstraint", CustomTypes::TYPE_PHYSUSERCONSTRAINT);
	LUA->SetMetaTable(-2);
}

CLuaUserConstraint *Get_UserConstraint(lua_State *state, int stackPos) {
	LUA->CheckType(stackPos, CustomTypes::TYPE_PHYSUSERCONSTRAINT);

	UserData *ud = (UserData *)LUA->GetUserdata(stackPos);
	return (CLuaUserConstraint *)ud->data;
}

//
// Name: constraint.User
// Desc: Links 2 objects together with a user constraint
// Arg1: PhysEnv|env|Physics environment
// Arg2: PhysObj|obj1|Physics object 1
// Arg3: PhysObj|obj2|Physics object 2
// Arg4: Table|constraint|User constraint (with GetConstraintInfo and GetConstraintSolveInfo functions)
// Ret1: 
//
int lConstraintUser(lua_State *state) {
	IPhysicsEnvironment32 *pEnv = Get_PhysEnv(state, 1);
	IPhysicsObject *pObjectA = Get_PhysObj(state, 2);
	IPhysicsObject *pObjectB = Get_PhysObj(state, 3);
	LUA->CheckType(4, GarrysMod::Lua::Type::TABLE);

	LUA->GetField(4, "GetConstraintInfo");
	LUA->CheckType(-1, GarrysMod::Lua::Type::FUNCTION);
	LUA->Pop();

	LUA->GetField(4, "GetConstraintSolveInfo");
	LUA->CheckType(-1, GarrysMod::Lua::Type::FUNCTION);
	LUA->Pop();

	// Create a reference of the table
	LUA->Push(4);
	int ref = LUA->ReferenceCreate();
	CLuaUserConstraint *pUserConstraint = new CLuaUserConstraint(LUA, state, ref);

	IPhysicsConstraint *pConstraint = pEnv->CreateUserConstraint(pObjectA, pObjectB, NULL, pUserConstraint);

	return 0;
}

int lPhysUserConstraint__index(lua_State *state) {
	return 0;
}

int Init_PhysConstraint(lua_State *state) {
	LUA->PushSpecial(GarrysMod::Lua::SPECIAL_GLOB);
		LUA->CreateTable();
			LUA->PushCFunction(lConstraintUser); LUA->SetField(-2, "User");
		LUA->SetField(-2, "CoolConstraint");
	LUA->Pop(1);

	// Returned constraint type
	LUA->CreateMetaTableType("PhysUserConstraint", CustomTypes::TYPE_PHYSUSERCONSTRAINT);
		LUA->Push(-1); LUA->SetField(-2, "__index");
	LUA->Pop();

	return 0;
}