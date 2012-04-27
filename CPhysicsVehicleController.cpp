#include "StdAfx.h"

#include "CPhysicsObject.h"
#include "CPhysicsVehicleController.h"
#include "CPhysicsEnvironment.h"

CPhysicsVehicleController::CPhysicsVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType)
{
	m_pEnv = pEnv;
	m_pBody = pBody;
	m_vehicleParams = params;
	m_iVehicleType = nVehicleType;

	m_pBody->GetObject()->setActivationState(DISABLE_DEACTIVATION);

	m_pRaycaster = new btDefaultVehicleRaycaster(m_pEnv->GetBulletEnvironment());
	m_pRaycastVehicle = new btRaycastVehicle(m_tuning, m_pBody->GetObject(), m_pRaycaster);
	m_pRaycastVehicle->setCoordinateSystem(0,1,2);
	m_pEnv->GetBulletEnvironment()->addVehicle(m_pRaycastVehicle);
}

CPhysicsVehicleController::~CPhysicsVehicleController()
{
}

void CPhysicsVehicleController::Update(float dt, vehicle_controlparams_t &controls)
{
}

float CPhysicsVehicleController::UpdateBooster(float dt)
{
	NOT_IMPLEMENTED;
	return 0.0f;
}

int CPhysicsVehicleController::GetWheelCount()
{
	return 0;
}

IPhysicsObject *CPhysicsVehicleController::GetWheel(int wheel)
{
	NOT_IMPLEMENTED;
	return m_pBody;
}

bool CPhysicsVehicleController::GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps)
{
	NOT_IMPLEMENTED;
	return false;
}

void CPhysicsVehicleController::SetSpringLength(int wheelIndex, float length)
{
	NOT_IMPLEMENTED;
}

void CPhysicsVehicleController::SetWheelFriction(int wheelIndex, float friction)
{
	NOT_IMPLEMENTED;
}

void CPhysicsVehicleController::OnVehicleEnter(void)
{
	NOT_IMPLEMENTED;
}

void CPhysicsVehicleController::OnVehicleExit(void)
{
	NOT_IMPLEMENTED;
}