#include "StdAfx.h"

#include "CPhysicsObject.h"
#include "CPhysicsVehicleController.h"
#include "CPhysicsEnvironment.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

CPhysicsVehicleController::CPhysicsVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace)
{
	m_pEnv = pEnv;
	m_pBody = pBody;
	m_vehicleParams = params;
	m_iVehicleType = nVehicleType;

	for (int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++) {
		m_pWheels[i] = NULL;
	}

	m_pBody->GetObject()->setActivationState(DISABLE_DEACTIVATION);

	m_pRaycaster = new btDefaultVehicleRaycaster(m_pEnv->GetBulletEnvironment());
	m_pRaycastVehicle = new btRaycastVehicle(m_tuning, m_pBody->GetObject(), m_pRaycaster);
	m_pRaycastVehicle->setCoordinateSystem(0,1,2);
	m_pEnv->GetBulletEnvironment()->addVehicle(m_pRaycastVehicle);

	m_iWheelCount = m_vehicleParams.axleCount * m_vehicleParams.wheelsPerAxle;

	// Initialization and setup
	InitCarWheels();
}

CPhysicsVehicleController::~CPhysicsVehicleController()
{
	m_pEnv->GetBulletEnvironment()->removeVehicle(m_pRaycastVehicle);

	delete m_pRaycaster;
	delete m_pRaycastVehicle;
}

void CPhysicsVehicleController::InitCarWheels()
{
	int wheelIndex = 0;

	for (int i = 0; i < m_vehicleParams.axleCount; i++) {
		for ( int w = 0; w < m_vehicleParams.wheelsPerAxle; w++, wheelIndex++ ) {
			CPhysicsObject *wheel = CreateWheel(wheelIndex, m_vehicleParams.axles[i]);
		}
	}
}

CPhysicsObject *CPhysicsVehicleController::CreateWheel(int wheelIndex, vehicle_axleparams_t &axle)
{
	if (wheelIndex >= VEHICLE_MAX_WHEEL_COUNT)
		return NULL;

	// TODO: bullet takes an argument bIsFrontWheel, should we use this, or does it matter if we do or don't?

	return NULL;
}

void CPhysicsVehicleController::Update(float dt, vehicle_controlparams_t &controls)
{
	float flThrottle = controls.throttle;
	bool bHandbrake = controls.handbrake;
	float flBrake = controls.brake;
	bool bPowerSlide = bHandbrake;

	if ( bPowerSlide ) {
		flThrottle = 0.0f;
	}

	if ( flThrottle == 0.0f && flBrake == 0.0f && !bHandbrake ) {
		flBrake = 0.1f;
	}

	UpdateSteering(dt, controls);

	// HELP ME, THE SPAM, IT'S TOO MUCH!
	//NOT_IMPLEMENTED
}

float CPhysicsVehicleController::UpdateBooster(float dt)
{
	NOT_IMPLEMENTED
	return 0.0f;
}

void CPhysicsVehicleController::UpdateSteering(float dt, vehicle_controlparams_t &controls)
{
	float flSteeringAngle = CalcSteering(dt, controls.steering);
	//NOT_IMPLEMENTED
}

float CPhysicsVehicleController::CalcSteering(float dt, float steering)
{
	// TODO: Determine angle based on vehicle speed
	// steering *= m_vehicleParams.steering.degre;
	return steering;
}

int CPhysicsVehicleController::GetWheelCount()
{
	return m_iWheelCount;
}

IPhysicsObject *CPhysicsVehicleController::GetWheel(int index)
{
	// if (m_iVehicleType = VEHICLE_TYPE_CAR_WHEELS) {
	// 	return m_pWheels[index];
	// }

	NOT_IMPLEMENTED
	return m_pBody;
}

bool CPhysicsVehicleController::GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps)
{
	NOT_IMPLEMENTED
	return false;
}

void CPhysicsVehicleController::SetSpringLength(int wheelIndex, float length)
{
	NOT_IMPLEMENTED
}

void CPhysicsVehicleController::SetWheelFriction(int wheelIndex, float friction)
{
	NOT_IMPLEMENTED
}

//--------------
// Debug stuff
//--------------
void CPhysicsVehicleController::GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem) {
	memset(&debugCarSystem, 0, sizeof(debugCarSystem));
}

//---------------------------------------
// Class factory
//---------------------------------------
IPhysicsVehicleController *CreateVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace)
{
	CPhysicsVehicleController *pController = new CPhysicsVehicleController(pEnv, pBody, params, nVehicleType, pGameTrace);
	//pController->InitCarSystem(pBody);

	return pController;
}