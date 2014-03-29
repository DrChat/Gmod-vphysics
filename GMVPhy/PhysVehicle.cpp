#include "PhysVehicle.h"

#include <GarrysMod/Lua/Interface.h>
#include <vphysics_interface.h>
#include "../include/vphysics_interfaceV32.h"
#include <vphysics/vehicles.h>

#include "MiscFuncs.h"

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

int Init_PhysVehicle(lua_State *state) {
	LUA->CreateMetaTableType("PhysVehicleController", CustomTypes::TYPE_PHYSVEHICLECONTROLLER);
		LUA->Push(-1); LUA->SetField(-2, "__index");
		LUA->PushCFunction(lPhysVehicleGetOperatingParams); LUA->SetField(-2, "GetOperatingParams");
	LUA->Pop();

	return 0;
}