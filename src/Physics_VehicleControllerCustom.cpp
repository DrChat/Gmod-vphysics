#include "StdAfx.h"

#include "Physics_Environment.h"
#include "Physics_Object.h"
#include "Physics_VehicleControllerCustom.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/*****************************************
* CLASS CPhysicsVehicleControllerCustom
*****************************************/

CPhysicsVehicleControllerCustom::CPhysicsVehicleControllerCustom(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, IPhysicsGameTrace *pGameTrace) {
	m_pEnv			= pEnv;
	m_pBody			= pBody;
	m_pGameTrace	= pGameTrace;
}

CPhysicsVehicleControllerCustom::~CPhysicsVehicleControllerCustom() {

}