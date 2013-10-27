#include "StdAfx.h"

#include "Physics_Environment.h"
#include "Physics_Object.h"
#include "Physics_VehicleControllerCustom.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

class CRaycastVehicle : public btRaycastVehicle {
	public:

};

/*****************************************
 * CLASS CPhysicsVehicleControllerCustom
 *****************************************/

CPhysicsVehicleControllerCustom::CPhysicsVehicleControllerCustom(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, IPhysicsGameTrace *pGameTrace) {
	m_pEnv = pEnv;
	m_pBody = pBody;
	m_pGameTrace = pGameTrace;
}

CPhysicsVehicleControllerCustom::~CPhysicsVehicleControllerCustom() {

}

void CPhysicsVehicleControllerCustom::SetWheelForce(int wheelIndex, float force) {

}

void CPhysicsVehicleControllerCustom::SetWheelBrake(int wheelIndex, float brakeVal) {

}

void CPhysicsVehicleControllerCustom::SetWheelSteering(int wheelIndex, float steerVal) {

}

void CPhysicsVehicleControllerCustom::Update(float dt) {

}

void CPhysicsVehicleControllerCustom::CreateWheel(const vehicle_customwheelparams_t& params) {

}