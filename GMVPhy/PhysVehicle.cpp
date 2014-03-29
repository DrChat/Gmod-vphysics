#include "PhysVehicle.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"
#include <vphysics/vehicles.h>

#include "MiscFuncs.h"

extern IPhysicsSurfaceProps *g_pSurfProps;

using namespace GarrysMod::Lua;

int lPhysVehicleGetOperatingParams(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	const vehicle_operatingparams_t &params = pController->GetOperatingParams();

	LUA->CreateTable();
		LUA->PushNumber(params.speed);
		LUA->SetField(-2, "speed");

		LUA->PushNumber(params.engineRPM);
		LUA->SetField(-2, "engineRPM");

		LUA->PushNumber(params.gear);
		LUA->SetField(-2, "gear");

		LUA->PushNumber(params.steeringAngle);
		LUA->SetField(-2, "steeringAngle");

		LUA->PushNumber(params.wheelsInContact);
		LUA->SetField(-2, "wheelsInContact");

		LUA->PushNumber(params.wheelsNotInContact);
		LUA->SetField(-2, "wheelsNotInContact");

		LUA->PushNumber(params.isTorqueBoosting);
		LUA->SetField(-2, "isTorqueBoosting");
	
	return 1;
}

int lPhysVehicleGetVehicleParams(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	const vehicleparams_t &params = pController->GetVehicleParams();

	// Jesus fucking long function
	// Outer table
	LUA->CreateTable();
		LUA->PushNumber(params.axleCount);
		LUA->SetField(-2, "axleCount");

		// Axles (outer table)
		LUA->CreateTable();
		for (int i = 0; i < params.axleCount; i++) {
			// This should not happen!
			Assert(i < VEHICLE_MAX_AXLE_COUNT);
			if (i >= VEHICLE_MAX_AXLE_COUNT) {
				continue;
			}

			const vehicle_axleparams_t &axle = params.axles[i];

			// Inner table
			LUA->PushNumber(i); // key for SetTable
			LUA->CreateTable();
				LUA->PushNumber(axle.brakeFactor);
				LUA->SetField(-2, "brakeFactor");

				Push_Vector(state, axle.offset);
				LUA->SetField(-2, "offset");

				LUA->PushNumber(axle.torqueFactor);
				LUA->SetField(-2, "torqueFactor");

				LUA->PushNumber(axle.suspension.springConstant);
				LUA->SetField(-2, "springConstant");

				LUA->PushNumber(axle.wheels.springAdditionalLength);
				LUA->SetField(-2, "springAdditionalLength");

				LUA->PushNumber(axle.wheels.frictionScale);
				LUA->SetField(-2, "frictionScale");

			LUA->SetTable(-3);
		}

	return 1;
}

int lPhysVehicleGetWheelMaterial(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	LUA->CheckType(2, Type::NUMBER);

	IPhysicsObject *pWheel = pController->GetWheel(LUA->GetNumber(2));
	if (!pWheel)
		return 0;

	const char *mat = g_pSurfProps->GetPropName(pWheel->GetMaterialIndex());
	LUA->PushString(mat);
	return 1;
}

int lPhysVehicleSetWheelMaterial(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	LUA->CheckType(2, Type::NUMBER);
	LUA->CheckType(3, Type::STRING);

	IPhysicsObject *pWheel = pController->GetWheel(LUA->GetNumber(2));
	if (!pWheel) {
		LUA->ThrowError("Invalid wheel index given");
		return 0;
	}

	int matId = g_pSurfProps->GetSurfaceIndex(LUA->GetString(3));
	if (matId == -1) {
		LUA->ThrowError("Invalid surface prop name");
		return 0;
	}

	pWheel->SetMaterialIndex(matId);
	return 0;
}

int Init_PhysVehicle(lua_State *state) {
	LUA->CreateMetaTableType("PhysVehicleController", CustomTypes::TYPE_PHYSVEHICLECONTROLLER);
		LUA->Push(-1); LUA->SetField(-2, "__index");
		LUA->PushCFunction(lPhysVehicleGetOperatingParams); LUA->SetField(-2, "GetOperatingParams");
		LUA->PushCFunction(lPhysVehicleGetWheelMaterial); LUA->SetField(-2, "GetWheelMaterial");
		LUA->PushCFunction(lPhysVehicleSetWheelMaterial); LUA->SetField(-2, "SetWheelMaterial");
	LUA->Pop();

	return 0;
}