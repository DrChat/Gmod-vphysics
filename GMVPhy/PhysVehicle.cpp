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

		LUA->PushNumber(params.boostDelay);
		LUA->SetField(-2, "boostDelay");

		LUA->PushNumber(params.boostTimeLeft);
		LUA->SetField(-2, "boostTimeLeft");

		LUA->PushNumber(params.skidSpeed);
		LUA->SetField(-2, "skidSpeed");

		LUA->PushNumber(params.skidMaterial);
		LUA->SetField(-2, "skidMaterial");

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

int lPhysVehicleSetVehicleParams(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	LUA->CheckType(2, Type::TABLE);

	LUA->GetField(2, "__TableCheck");
	if (!LUA->GetBool()) {
		LUA->ThrowError("Use the table provided from GetVehicleParams!");
	}
	LUA->Pop();

	vehicleparams_t &params = pController->GetVehicleParamsForChange();
	// TODO: Is there a faster way to generate code to read the stupid table?
	return 0;
}

int lPhysVehicleGetVehicleParams(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	const vehicleparams_t &params = pController->GetVehicleParams();

	// Jesus fucking long function
	// Outer table
	LUA->CreateTable();
		LUA->PushBool(true);
		LUA->SetField(-2, "__TableCheck");

		LUA->PushNumber(params.axleCount);
		LUA->SetField(-2, "axleCount");

		LUA->PushNumber(params.wheelsPerAxle);
		LUA->SetField(-2, "wheelsPerAxle");

		// Body
		LUA->CreateTable();
			Push_Vector(state, params.body.massCenterOverride);
			LUA->SetField(-2, "massCenterOverride");

			LUA->PushNumber(params.body.massOverride);
			LUA->SetField(-2, "massOverride");

			LUA->PushNumber(params.body.addGravity);
			LUA->SetField(-2, "addGravity");

			LUA->PushNumber(params.body.tiltForce);
			LUA->SetField(-2, "tiltForce");

			LUA->PushNumber(params.body.tiltForceHeight);
			LUA->SetField(-2, "tiltForceHeight");

			LUA->PushNumber(params.body.counterTorqueFactor);
			LUA->SetField(-2, "counterTorqueFactor");

			LUA->PushNumber(params.body.keepUprightTorque);
			LUA->SetField(-2, "keepUprightTorque");

			LUA->PushNumber(params.body.maxAngularVelocity);
			LUA->SetField(-2, "maxAngularVelocity");
		LUA->SetField(-2, "body");

		// Axles (outer table)
		LUA->CreateTable();
		for (int i = 0; i < params.axleCount; i++) {
			// This should not happen!
			Assert(i < VEHICLE_MAX_AXLE_COUNT);
			if (i >= VEHICLE_MAX_AXLE_COUNT) {
				break;
			}

			const vehicle_axleparams_t &axle = params.axles[i];

			// Inner table
			LUA->PushNumber(i); // key for SetTable
			LUA->CreateTable();
				Push_Vector(state, axle.offset);
				LUA->SetField(-2, "offset");

				Push_Vector(state, axle.wheelOffset);
				LUA->SetField(-2, "wheelOffset");

				Push_Vector(state, axle.raytraceCenterOffset);
				LUA->SetField(-2, "raytraceCenterOffset");

				Push_Vector(state, axle.raytraceOffset);
				LUA->SetField(-2, "raytraceOffset");

				LUA->PushNumber(axle.torqueFactor);
				LUA->SetField(-2, "torqueFactor");

				LUA->PushNumber(axle.brakeFactor);
				LUA->SetField(-2, "brakeFactor");

				// Wheels
				LUA->CreateTable();
					LUA->PushNumber(axle.wheels.radius);
					LUA->SetField(-2, "radius");

					LUA->PushNumber(axle.wheels.mass);
					LUA->SetField(-2, "mass");

					LUA->PushNumber(axle.wheels.inertia);
					LUA->SetField(-2, "inertia");

					LUA->PushNumber(axle.wheels.damping);
					LUA->SetField(-2, "damping");

					LUA->PushNumber(axle.wheels.frictionScale);
					LUA->SetField(-2, "frictionScale");

					LUA->PushNumber(axle.wheels.materialIndex);
					LUA->SetField(-2, "materialIndex");

					LUA->PushNumber(axle.wheels.brakeMaterialIndex);
					LUA->SetField(-2, "brakeMaterialIndex");

					LUA->PushNumber(axle.wheels.skidMaterialIndex);
					LUA->SetField(-2, "skidMaterialIndex");

					LUA->PushNumber(axle.wheels.springAdditionalLength);
					LUA->SetField(-2, "springAdditionalLength");
				LUA->SetField(-2, "wheels");

				// Suspension
				LUA->CreateTable();
					LUA->PushNumber(axle.suspension.springConstant);
					LUA->SetField(-2, "springConstant");

					LUA->PushNumber(axle.suspension.springDamping);
					LUA->SetField(-2, "springDamping");

					LUA->PushNumber(axle.suspension.stabilizerConstant);
					LUA->SetField(-2, "stabilizerConstant");

					LUA->PushNumber(axle.suspension.springDampingCompression);
					LUA->SetField(-2, "springDampingCompression");

					LUA->PushNumber(axle.suspension.maxBodyForce);
					LUA->SetField(-2, "maxBodyForce");
				LUA->SetField(-2, "suspension");

			LUA->SetTable(-3);
		}
		LUA->SetField(-2, "axles");

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

int lPhysVehicleGetWheel(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	LUA->CheckType(2, Type::NUMBER);

	IPhysicsObject *pWheel = pController->GetWheel(LUA->GetNumber(2));
	Push_PhysObj(state, pWheel);
	return 1;
}

int lPhysVehicleSetSpringLength(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	LUA->CheckType(2, Type::NUMBER);
	LUA->CheckType(3, Type::NUMBER);

	int wheel = LUA->GetNumber(2);
	if (wheel < 0 || wheel >= pController->GetWheelCount())
		LUA->ThrowError("Wheel index is out of bounds!");

	pController->SetSpringLength(wheel, LUA->GetNumber(3));
	return 0;
}

int lPhysVehicleSetWheelFriction(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	LUA->CheckType(2, Type::NUMBER);
	LUA->CheckType(3, Type::NUMBER);

	int wheel = LUA->GetNumber(2);
	if (wheel < 0 || wheel >= pController->GetWheelCount())
		LUA->ThrowError("Wheel index is out of bounds!");

	pController->SetWheelFriction(wheel, LUA->GetNumber(3));
	return 0;
}

//
// Name: PhysVehicle:GetWheelContactPoint
// Desc: Get wheel contact point
// Arg1: PhysVehicle|vehicle|The vehicle
// Arg2: int|wheel|Wheel index
// Ret1: any|contact|Table contact details {contact=Vector(), surfaceProps=int}, or nil if not in contact
//
int lPhysVehicleGetWheelContactPoint(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	LUA->CheckType(2, Type::NUMBER);

	int wheel = LUA->GetNumber(2);
	if (wheel < 0 || wheel >= pController->GetWheelCount())
		LUA->ThrowError("Wheel index is out of bounds!");

	Vector contact(0, 0, 0);
	int surfProps = 0;
	if (pController->GetWheelContactPoint(wheel, &contact, &surfProps)) {
		LUA->CreateTable();
		Push_Vector(state, contact);
		LUA->SetField(-2, "contact");

		LUA->PushNumber(surfProps);
		LUA->SetField(-2, "surfaceProps");

		return 1;
	} else {
		LUA->PushNil();
		return 1;
	}
}

int lPhysVehicleDataReload(lua_State *state) {
	IPhysicsVehicleController *pController = Get_PhysVehicleController(state, 1);
	pController->VehicleDataReload();
	return 0;
}

int Init_PhysVehicle(lua_State *state) {
	LUA->CreateMetaTableType("PhysVehicleController", CustomTypes::TYPE_PHYSVEHICLECONTROLLER);
		LUA->Push(-1); LUA->SetField(-2, "__index");
		LUA->PushCFunction(lPhysVehicleGetOperatingParams);	LUA->SetField(-2, "GetOperatingParams");
		LUA->PushCFunction(lPhysVehicleGetWheelMaterial);	LUA->SetField(-2, "GetWheelMaterial");
		LUA->PushCFunction(lPhysVehicleSetWheelMaterial);	LUA->SetField(-2, "SetWheelMaterial");
		LUA->PushCFunction(lPhysVehicleGetWheel);			LUA->SetField(-2, "GetWheel");
		LUA->PushCFunction(lPhysVehicleSetSpringLength);	LUA->SetField(-2, "SetSpringLength");
		LUA->PushCFunction(lPhysVehicleSetWheelFriction);	LUA->SetField(-2, "SetWheelFriction");
		LUA->PushCFunction(lPhysVehicleGetWheelContactPoint); LUA->SetField(-2, "GetWheelContactPoint");
		LUA->PushCFunction(lPhysVehicleDataReload);			LUA->SetField(-2, "VehicleDataReload");
	LUA->Pop();

	return 0;
}