#include "StdAfx.h"

#include "CPhysicsObject.h"
#include "CPhysicsVehicleController.h"
#include "CPhysicsEnvironment.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

// MPH2INS: 1(miles/hr) = 0.44704(meters/sec)
#define MPH2MS(x) ((x) * 0.44704)

CPhysicsVehicleController::CPhysicsVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	m_pEnv = pEnv;
	m_pBody = pBody;
	m_iVehicleType = nVehicleType;
	memset(&m_vehicleState, 0, sizeof(m_vehicleState));
	InitVehicleParams(params);

	for (int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++) {
		m_pWheels[i] = NULL;
	}

	m_pBody->GetObject()->setActivationState(DISABLE_DEACTIVATION);
	m_iWheelCount = m_vehicleParams.axleCount * m_vehicleParams.wheelsPerAxle;

	// Initialization and setup
	InitBullVehicle();
	InitCarWheels();
}

CPhysicsVehicleController::~CPhysicsVehicleController() {
	ShutdownBullVehicle();
}

void CPhysicsVehicleController::InitVehicleParams(const vehicleparams_t &params) {
	m_vehicleParams = params;
	m_vehicleParams.engine.maxSpeed			= MPH2MS(params.engine.maxSpeed);
	m_vehicleParams.engine.maxRevSpeed		= MPH2MS(params.engine.maxRevSpeed);
	m_vehicleParams.engine.boostMaxSpeed	= MPH2MS(params.engine.boostMaxSpeed);
	m_vehicleParams.body.tiltForceHeight	= ConvertDistanceToBull(params.body.tiltForceHeight);
	for (int i = 0; i < m_vehicleParams.axleCount; i++) {
		m_vehicleParams.axles[i].wheels.radius = ConvertDistanceToBull(params.axles[i].wheels.radius);
		m_vehicleParams.axles[i].wheels.springAdditionalLength = ConvertDistanceToBull(params.axles[i].wheels.springAdditionalLength);
	}
}

void CPhysicsVehicleController::InitBullVehicle() {
	m_pRaycaster = new btDefaultVehicleRaycaster(m_pEnv->GetBulletEnvironment());
	m_pRaycastVehicle = new btRaycastVehicle(m_tuning, m_pBody->GetObject(), m_pRaycaster);
	m_pRaycastVehicle->setCoordinateSystem(0,1,2);
	m_pEnv->GetBulletEnvironment()->addVehicle(m_pRaycastVehicle);
}

void CPhysicsVehicleController::InitCarWheels() {
	int wheelIndex = 0;

	for (int i = 0; i < m_vehicleParams.axleCount; i++) {
		for ( int w = 0; w < m_vehicleParams.wheelsPerAxle; w++, wheelIndex++ ) {
			CPhysicsObject *pWheel = CreateWheel(wheelIndex, m_vehicleParams.axles[i]);
			if (pWheel) {
				m_pWheels[i] = pWheel;
			}
		}
	}

	// TODO: Disable collisions between two objects
	// See: http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=4853
	for (int i = 0; i < m_iWheelCount; i++) {
		m_pWheels[i]->EnableCollisions(true);
	}
}

// Purpose: Create wheel on source side (CPhysicsObject *)
CPhysicsObject *CPhysicsVehicleController::CreateWheel(int wheelIndex, vehicle_axleparams_t &axle) {
	if (wheelIndex >= VEHICLE_MAX_WHEEL_COUNT)
		return NULL;

	Vector position = axle.offset;

	Vector bodyPosition;
	QAngle bodyAngles;
	m_pBody->GetPosition( &bodyPosition, &bodyAngles );
	matrix3x4_t matrix;
	AngleMatrix( bodyAngles, bodyPosition, matrix );

	// Note: This will only work with vehicles that have 2 wheels per axle
	if (wheelIndex & 1) {
		position += axle.wheelOffset;
	} else {
		position -= axle.wheelOffset;
	}

	QAngle angles = vec3_angle;
	Vector wheelPositionHL;
	VectorTransform( position, matrix, wheelPositionHL );

	objectparams_t params;
	memset(&params, 0, sizeof(params));

	params.damping = axle.wheels.damping;
	params.dragCoefficient = 0;
	params.enableCollisions = false;
	params.inertia = axle.wheels.inertia;
	params.mass = axle.wheels.mass;
	params.pGameData = m_pBody->GetGameData();
	params.pName = "VehicleWheel";
	params.rotdamping = axle.wheels.rotdamping;
	params.rotInertiaLimit = 0;
	params.massCenterOverride = NULL;
	// needs to be in HL units because we're calling through the "outer" interface to create
	// the wheels
	float radius = ConvertDistanceToHL( axle.wheels.radius );
	float r3 = radius * radius * radius;
	params.volume = (4 / 3) * M_PI * r3;

	CPhysicsObject *pWheel = (CPhysicsObject *)m_pEnv->CreateSphereObject(radius, axle.wheels.materialIndex, wheelPositionHL, angles, &params, false);
	pWheel->Wake();

	// cache the wheel object pointer
	m_pWheels[wheelIndex] = pWheel;

	pWheel->SetCallbackFlags( pWheel->GetCallbackFlags() | CALLBACK_IS_VEHICLE_WHEEL );

	// Create the wheel in bullet
	btVector3 bullConnectionPointCS0;
	btScalar bullSuspensionRestLength, bullWheelRadius;

	btVector3 bullWheelDirectionCS0(0,-1,0);	// TODO: Figure out what this is.
	btVector3 bullWheelAxleCS(-1,0,0);			// TODO: Figure out what this is.

	position += Vector(0, 35, -42);	// The wheels are spawned too high!
	ConvertPosToBull(position, bullConnectionPointCS0);
	bullSuspensionRestLength = ConvertDistanceToBull(axle.suspension.springConstant + axle.wheels.springAdditionalLength);
	bullWheelRadius = ConvertDistanceToBull(axle.wheels.radius);
	bool bIsFrontWheel = (wheelIndex < 2); // NOTE: Only works with 2 front wheels

	btWheelInfo wheelInfo = m_pRaycastVehicle->addWheel(bullConnectionPointCS0, bullWheelDirectionCS0, bullWheelAxleCS, bullSuspensionRestLength, bullWheelRadius, m_tuning, bIsFrontWheel);

	wheelInfo.m_maxSuspensionForce = axle.suspension.maxBodyForce;
	wheelInfo.m_frictionSlip = axle.wheels.frictionScale;			// TODO: How do we convert this?

	//wheel.m_suspensionStiffness = suspensionStiffness;
	//wheel.m_wheelsDampingRelaxation = suspensionDamping;
	//wheel.m_wheelsDampingCompression = suspensionCompression;
	//wheel.m_frictionSlip = wheelFriction;
	//wheel.m_rollInfluence = rollInfluence;

	return pWheel;
}

void CPhysicsVehicleController::ShutdownBullVehicle() {
	m_pEnv->GetBulletEnvironment()->removeVehicle(m_pRaycastVehicle);

	delete m_pRaycaster;
	delete m_pRaycastVehicle;

	for (int i = 0; i < m_iWheelCount; i++) {
		m_pEnv->DestroyObject(m_pWheels[i]);
		m_pWheels[i] = NULL;
	}
}

void CPhysicsVehicleController::Update(float dt, vehicle_controlparams_t &controls) {
	if ( controls.handbrake )
	{
		controls.throttle = 0.0f;
	}

	if ( controls.throttle == 0.0f && controls.brake == 0.0f && !controls.handbrake )
	{
		controls.brake = 0.1f;
	}

	UpdateSteering(controls, dt);
	UpdateEngine(controls, dt);
	UpdateWheels(controls, dt);

	m_pRaycastVehicle->updateVehicle(dt);
}

void CPhysicsVehicleController::UpdateSteering(const vehicle_controlparams_t &controls, float dt) {
	float steeringVal = controls.steering;

	btScalar bullCurrentSpeed = m_pRaycastVehicle->getCurrentSpeedKmHour();
	bullCurrentSpeed = bullCurrentSpeed / 3.6; // Convert from km/h to m/s

	// TODO: Calculate for degreesSlow, degreesFast, and degreesBoost
	steeringVal *= m_vehicleParams.steering.degreesFast;

	m_vehicleState.steeringAngle = steeringVal;

	// BUG: Only works for vehicles with 2 front wheels.
	for (int i = 0; i < 2; i++) {
		m_pRaycastVehicle->setSteeringValue(DEG2RAD(-steeringVal), i);
	}
}

void CPhysicsVehicleController::UpdateEngine(const vehicle_controlparams_t &controls, float dt) {
	// Update the operating params
	float fSpeed = m_pRaycastVehicle->getCurrentSpeedKmHour();
	fSpeed *= 0.27777778; // km/h -> m/s
	m_vehicleState.speed = ConvertDistanceToHL(-fSpeed);

	for (int i = 2; i < m_iWheelCount; i++) {
		m_pRaycastVehicle->applyEngineForce(-controls.throttle * 1000, i);
		m_pRaycastVehicle->setBrake(0, i);
	}
}

void CPhysicsVehicleController::UpdateWheels(const vehicle_controlparams_t &controls, float dt) {
	for (int i = 0; i < m_iWheelCount; i++) {
		btTransform bullTransform = m_pRaycastVehicle->getWheelTransformWS(i);
		btVector3 bullPos = bullTransform.getOrigin();
		btQuaternion bullRot = bullTransform.getRotation();

		Vector HLPos;
		QAngle HLRot;
		ConvertPosToHL(bullPos, HLPos);
		ConvertRotationToHL(bullRot, HLRot);

		m_pWheels[i]->SetPosition(HLPos, HLRot, true);
	}
}

float CPhysicsVehicleController::UpdateBooster(float dt) {
	NOT_IMPLEMENTED
	return 0.0f;		// Return boost delay.
}

int CPhysicsVehicleController::GetWheelCount() {
	return m_iWheelCount;
}

IPhysicsObject *CPhysicsVehicleController::GetWheel(int index) {
	if (m_iVehicleType = VEHICLE_TYPE_CAR_WHEELS) {
		return m_pWheels[index];
	}

	return NULL;
}

bool CPhysicsVehicleController::GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps) {
	if ((index >= m_iWheelCount || index < 0) || (!pContactPoint && !pSurfaceProps)) return false;

	btWheelInfo wheelInfo = m_pRaycastVehicle->getWheelInfo(index);
	if (wheelInfo.m_raycastInfo.m_isInContact) {
		btVector3 bullContactVec = wheelInfo.m_raycastInfo.m_contactPointWS;

		if (pContactPoint)
			ConvertPosToHL(bullContactVec, *pContactPoint);

		return true;
	}
	return false;
}

void CPhysicsVehicleController::SetSpringLength(int wheelIndex, float length) {
	NOT_IMPLEMENTED
}

void CPhysicsVehicleController::SetWheelFriction(int wheelIndex, float friction) {
	NOT_IMPLEMENTED
}

//--------------
// Debug stuff
//--------------
void CPhysicsVehicleController::GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem) {
	memset(&debugCarSystem, 0, sizeof(debugCarSystem));
	NOT_IMPLEMENTED
}

//---------------------------------------
// Class factory
//---------------------------------------
IPhysicsVehicleController *CreateVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	CPhysicsVehicleController *pController = new CPhysicsVehicleController(pEnv, pBody, params, nVehicleType, pGameTrace);
	//pController->InitCarSystem(pBody);

	return pController;
}