#include "StdAfx.h"

#include <cmodel.h>

#include "Physics_Object.h"
#include "Physics_VehicleController.h"
#include "Physics_Environment.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

// MPH2INS: 1(miles/hr) = 0.44704(meters/sec)
#define MPH2MS(x) ((x) * 0.44704)
// KMH2MS: 1(km/h) = 0.27777778(meters/sec)
#define KMH2MS(x) ((x) * 0.27777778)

/*********************************
* MISC CLASSES
*********************************/

class CDefaultCarWheelTracer : public IPhysicsVehicleWheelTrace {
	public:
		CDefaultCarWheelTracer(btRaycastVehicle *pVehicle);

		IPhysicsObject *CastRay(int wheelIndex, const Vector &start, const Vector &end, trace_t &result) {

		}
	private:
		btRaycastVehicle *m_pVehicle;
};

// Purpose: This ray will ignore a body AND detect water for use in airboats.
struct CDetectWaterRayResultCallback : public btCollisionWorld::ClosestRayResultCallback {
	CDetectWaterRayResultCallback(const btRigidBody *pIgnoreObject, const btVector3 &from, const btVector3 &to)
			: ClosestRayResultCallback(from, to) {
		m_pIgnoreObject = pIgnoreObject;
	}

	bool needsCollision(btBroadphaseProxy *proxy0) const {
		btRigidBody *pBody = (btRigidBody *)proxy0->m_clientObject;
		if (pBody) {
			CPhysicsObject *pPhys = (CPhysicsObject *)pBody->getUserPointer();

			if (pBody == m_pIgnoreObject)
				return false;

			if (pPhys) {
				if (pPhys->GetCallbackFlags() & CALLBACK_FLUID_TOUCH || pPhys->GetContents() & MASK_WATER)
					return true;
			}
		}

		return btCollisionWorld::ClosestRayResultCallback::needsCollision(proxy0);
	}

	const btRigidBody *m_pIgnoreObject;
};

// Purpose: Airboat raycaster
class CAirboatRaycaster : public btVehicleRaycaster {
	public:
		CAirboatRaycaster(btDynamicsWorld *pWorld, btRigidBody *pBody) {
			m_pWorld = pWorld;
			m_pBody = pBody;
		}

		// Returns the rigid body the ray hits
		void *castRay(const btVector3 &from, const btVector3 &to, btVehicleRaycasterResult &result) {
			CDetectWaterRayResultCallback rayCallback(m_pBody, from, to);

			m_pWorld->rayTest(from, to, rayCallback);

			if (rayCallback.hasHit()) {
				const btRigidBody *body = btRigidBody::upcast(rayCallback.m_collisionObject);
				if (body) {
					result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
					result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
					result.m_hitNormalInWorld.normalize();
					result.m_distFraction = rayCallback.m_closestHitFraction;
					return (void *)body;
				}
			}

			return NULL;
		}
	private:
		btDynamicsWorld *	m_pWorld;
		btRigidBody *		m_pBody;
};

struct CIgnoreObjectRayResultCallback : public btCollisionWorld::ClosestRayResultCallback {
	CIgnoreObjectRayResultCallback(const btRigidBody *pIgnoreObject, const btVector3 &from, const btVector3 &to)
			: ClosestRayResultCallback(from, to) {
		m_pIgnoreObject = pIgnoreObject;
	}

	bool needsCollision(btBroadphaseProxy *proxy0) const {
		btRigidBody *pBody = (btRigidBody *)proxy0->m_clientObject;
		CPhysicsObject *pPhys = (CPhysicsObject *)pBody->getUserPointer();

		if (pBody && pBody == m_pIgnoreObject) {
			return false;
		} else if (pPhys && !pPhys->IsCollisionEnabled()) {
			return false;
		}

		return btCollisionWorld::ClosestRayResultCallback::needsCollision(proxy0);
	}

	const btRigidBody *m_pIgnoreObject;
};

// Purpose: This raycaster will cast a ray ignoring the vehicle's body.
class CCarRaycaster : public btVehicleRaycaster {
	public:
		CCarRaycaster(btDynamicsWorld *pWorld, CPhysicsVehicleController *pController) {
			m_pWorld = pWorld;
			m_pController = pController;
		}

		void *castRay(const btVector3 &from, const btVector3 &to, btVehicleRaycasterResult &result) {
			CIgnoreObjectRayResultCallback rayCallback(m_pController->GetBody()->GetObject(), from, to);
			
			m_pWorld->rayTest(from, to, rayCallback);
			
			if (rayCallback.hasHit()) {
				const btRigidBody *body = btRigidBody::upcast(rayCallback.m_collisionObject);
				if (body && body->hasContactResponse()) {
					result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
					result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
					result.m_hitNormalInWorld.normalize();
					result.m_distFraction = rayCallback.m_closestHitFraction;
					return (void *)body;
				}
			}

			return NULL;
		}
	private:
		btDynamicsWorld *			m_pWorld;
		CPhysicsVehicleController *	m_pController;
};

/*********************************
* CLASS CPhysicsVehicleController
*********************************/

CPhysicsVehicleController::CPhysicsVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	m_pEnv				= pEnv;
	m_pBody				= pBody;
	m_iVehicleType		= nVehicleType;
	m_pGameTrace		= pGameTrace;
	m_bEngineDisabled	= false;

	memset(&m_vehicleState, 0, sizeof(m_vehicleState));
	InitVehicleParams(params);

	for (int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++) {
		m_pWheels[i] = NULL;
	}

	m_pBody->GetObject()->setActivationState(DISABLE_DEACTIVATION);
	m_iWheelCount = m_vehicleParams.axleCount * m_vehicleParams.wheelsPerAxle;

	// Initialization and setup
	InitBullVehicle();
}

CPhysicsVehicleController::~CPhysicsVehicleController() {
	ShutdownBullVehicle();
}

void CPhysicsVehicleController::InitVehicleParams(const vehicleparams_t &params) {
	m_vehicleParams							= params;
	m_vehicleParams.engine.maxSpeed			= MPH2MS(params.engine.maxSpeed);
	m_vehicleParams.engine.maxRevSpeed		= MPH2MS(params.engine.maxRevSpeed);
	m_vehicleParams.engine.boostMaxSpeed	= MPH2MS(params.engine.boostMaxSpeed);
	m_vehicleParams.body.tiltForceHeight	= ConvertDistanceToBull(params.body.tiltForceHeight);

	for (int i = 0; i < m_vehicleParams.axleCount; i++) {
		m_vehicleParams.axles[i].wheels.radius					= ConvertDistanceToBull(params.axles[i].wheels.radius);
		m_vehicleParams.axles[i].wheels.springAdditionalLength	= ConvertDistanceToBull(params.axles[i].wheels.springAdditionalLength);
	}
}

void CPhysicsVehicleController::InitBullVehicle() {
	// NOTE: We're faking the car wheels for now because bullet does not offer a vehicle with physical wheels.
	// TODO: Simulate the car wheels to a degree
	// See: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=&f=&t=1817
	if (m_iVehicleType == VEHICLE_TYPE_CAR_WHEELS)
		m_pRaycaster = new CCarRaycaster(m_pEnv->GetBulletEnvironment(), this);
	else if (m_iVehicleType == VEHICLE_TYPE_CAR_RAYCAST)
		m_pRaycaster = new CCarRaycaster(m_pEnv->GetBulletEnvironment(), this);
	else if (m_iVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST)
		m_pRaycaster = new CAirboatRaycaster(m_pEnv->GetBulletEnvironment(), m_pBody->GetObject());
	else
		Assert(0);

	m_pRaycastVehicle = new btRaycastVehicle(m_tuning, m_pBody->GetObject(), m_pRaycaster);
	m_pRaycastVehicle->setCoordinateSystem(0, 1, 2);
	m_pEnv->GetBulletEnvironment()->addAction(m_pRaycastVehicle);

	InitCarWheels();
}

void CPhysicsVehicleController::ShutdownBullVehicle() {
	m_pEnv->GetBulletEnvironment()->removeAction(m_pRaycastVehicle);

	for (int i = 0; i < m_iWheelCount; i++) {
		m_pEnv->DestroyObject(m_pWheels[i]);
		m_pWheels[i] = NULL;
	}

	delete m_pRaycaster;
	delete m_pRaycastVehicle;
}

void CPhysicsVehicleController::InitCarWheels() {
	int wheelIndex = 0;

	for (int i = 0; i < m_vehicleParams.axleCount; i++) {
		for (int w = 0; w < m_vehicleParams.wheelsPerAxle; w++, wheelIndex++) {
			CPhysicsObject *pWheel = CreateWheel(wheelIndex, m_vehicleParams.axles[i]);
			if (pWheel) {
				m_pWheels[wheelIndex] = pWheel;
			}
		}
	}

	for (int i = 0; i < m_iWheelCount; i++) {
		m_pWheels[i]->EnableGravity(false);	// Otherwise they slowly sink.
	}
}

// Purpose: Create wheel on source side (CPhysicsObject *) and add a wheel to the raycaster.
CPhysicsObject *CPhysicsVehicleController::CreateWheel(int wheelIndex, vehicle_axleparams_t &axle) {
	if (wheelIndex >= VEHICLE_MAX_WHEEL_COUNT)
		return NULL;

	Vector position = axle.offset;
	Vector bodyPosition;
	QAngle bodyAngles;
	m_pBody->GetPosition(&bodyPosition, &bodyAngles);
	matrix3x4_t matrix;
	AngleMatrix(bodyAngles, bodyPosition, matrix);

	// HACK: This will only work with vehicles that have 2 wheels per axle
	if (wheelIndex & 1) {
		position += axle.wheelOffset;
	} else {
		position -= axle.wheelOffset;
	}

	QAngle angles = vec3_angle;
	Vector wheelPositionHL;
	VectorTransform(position, matrix, wheelPositionHL);

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

	// needs to be in HL units because we're calling through the "outer" interface to create
	// the wheels
	float radius = ConvertDistanceToHL(axle.wheels.radius);
	float r3 = radius * radius * radius;
	params.volume = (4 / 3) * M_PI * r3;

	CPhysicsObject *pWheel = (CPhysicsObject *)m_pEnv->CreateSphereObject(radius, axle.wheels.materialIndex, wheelPositionHL, angles, &params);
	pWheel->Wake();
	pWheel->AddCallbackFlags(CALLBACK_IS_VEHICLE_WHEEL);

	// Create the wheel in bullet
	btVector3 bullConnectionPointCS0;
	btScalar bullSuspensionRestLength, bullWheelRadius;

	bool bIsFrontWheel = (wheelIndex < 2);		// HACK: Only works with 2 front wheels

	btVector3 bullWheelDirectionCS0(0, -1, 0);	// Straight down
	btVector3 bullWheelAxleCS(1, 0, 0);			// Left

	ConvertPosToBull(position, bullConnectionPointCS0);
	bullConnectionPointCS0 -= ((btMassCenterMotionState *)m_pBody->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();

	bullSuspensionRestLength = axle.wheels.springAdditionalLength;
	bullWheelRadius = axle.wheels.radius;

	btWheelInfo &wheelInfo = m_pRaycastVehicle->addWheel(bullConnectionPointCS0, bullWheelDirectionCS0, bullWheelAxleCS, bullSuspensionRestLength, bullWheelRadius, m_tuning, bIsFrontWheel);

	// FIXME: frictionScale is UNUSED (or we're not parsing something correctly)!
	//wheelInfo.m_frictionSlip = axle.wheels.frictionScale;
	wheelInfo.m_frictionSlip = 1.5f; // debug value
	wheelInfo.m_maxSuspensionForce = axle.suspension.maxBodyForce * m_pBody->GetMass();
	wheelInfo.m_suspensionStiffness = axle.suspension.springConstant;

	return pWheel;
}

void CPhysicsVehicleController::Update(float dt, vehicle_controlparams_t &controls) {
	if (controls.handbrake) {
		controls.throttle = 0.0f;
	}

	if (controls.throttle == 0.0f && controls.brake == 0.0f && !controls.handbrake) {
		controls.brake = 0.1f;
	}

	UpdateSteering(controls, dt);
	UpdateEngine(controls, dt);
	UpdateWheels(controls, dt);

	m_pRaycastVehicle->updateVehicle(dt);
}

void CPhysicsVehicleController::UpdateSteering(const vehicle_controlparams_t &controls, float dt) {
	float steeringVal = controls.steering;

	// TODO: Calculate for degreesSlow, degreesFast, and degreesBoost
	steeringVal *= m_vehicleParams.steering.degreesFast;
	m_vehicleState.steeringAngle = steeringVal;

	for (int i = 0; i < m_iWheelCount; i++) {
		if (m_pRaycastVehicle->getWheelInfo(i).m_bIsFrontWheel)
			m_pRaycastVehicle->setSteeringValue(DEG2RAD(-steeringVal), i);
	}
}

void CPhysicsVehicleController::UpdateEngine(const vehicle_controlparams_t &controls, float dt) {
	// Update the operating params
	float fSpeed = m_pRaycastVehicle->getCurrentSpeedKmHour();
	m_vehicleState.speed = ConvertDistanceToHL(KMH2MS(-fSpeed));

	CalcEngine(controls, dt);
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

		// z = spin
		// flip it because HL expects it to come in opposite for some reason.
		HLRot.z = -HLRot.z;

		m_pWheels[i]->SetPosition(HLPos, HLRot, true);
	}
}

float CPhysicsVehicleController::UpdateBooster(float dt) {
	NOT_IMPLEMENTED
	return 0.0f; // Return boost delay.
}

void CPhysicsVehicleController::CalcEngine(const vehicle_controlparams_t &controls, float dt) {
	// throttle goes forward and backward, [-1, 1]
	// brake_val [0..1]

	float absSpeed = fabs(KMH2MS(m_pRaycastVehicle->getCurrentSpeedKmHour()));

	if (m_vehicleParams.engine.isAutoTransmission) {
		// Estimate the engine RPM
		float avgRotSpeed = 0;
		for (int i = 0; i < m_iWheelCount; i++) {
			btWheelInfo wheelInfo = m_pRaycastVehicle->getWheelInfo(i);

			float rotSpeed = fabs(wheelInfo.m_deltaRotation / dt);
			avgRotSpeed += rotSpeed;
		}

		avgRotSpeed *= 0.5f / M_PI / m_iWheelCount;

		// Note: 60 = seconds per minute
		float estEngineRPM = avgRotSpeed * m_vehicleParams.engine.axleRatio * m_vehicleParams.engine.gearRatio[m_vehicleState.gear] * 60;

		// only shift up when going forward
		if (controls.throttle > 0) {
			// check for higher gear, top gear is gearcount-1 (0 based)
			while (estEngineRPM > m_vehicleParams.engine.shiftUpRPM && m_vehicleState.gear < m_vehicleParams.engine.gearCount-1) {
				m_vehicleState.gear++;
				estEngineRPM = avgRotSpeed * m_vehicleParams.engine.axleRatio * m_vehicleParams.engine.gearRatio[m_vehicleState.gear] * 60;
			}
		}

		// check for lower gear
		while (estEngineRPM < m_vehicleParams.engine.shiftDownRPM && m_vehicleState.gear > 0) {
			m_vehicleState.gear--;
			estEngineRPM = avgRotSpeed * m_vehicleParams.engine.axleRatio * m_vehicleParams.engine.gearRatio[m_vehicleState.gear] * 60;
		}

		m_vehicleState.engineRPM = estEngineRPM;
	}

	// Speed governor
	// TODO: These were apparently scrapped when vphysics was shipped. Figure out new speed governors by disassembly.

	// Apply our forces!
	// FIXME: Forces are in NEWTONS!
	if (fabs(controls.throttle) > 1e-4) {
		for (int i = 0; i < m_iWheelCount; i++) {
			m_pRaycastVehicle->setBrake(0, i);
		}

		const static float watt_per_hp = 745.0f;
		const static float seconds_per_minute = 60.0f;

		float brakeForce = controls.throttle * 
			m_vehicleParams.engine.horsepower * (watt_per_hp * seconds_per_minute) * 
			m_vehicleParams.engine.gearRatio[m_vehicleState.gear]  * m_vehicleParams.engine.axleRatio / 
			(m_vehicleParams.engine.maxRPM * (2 * M_PI));

		int wheelIndex = 0;
		for (int i = 0; i < m_vehicleParams.axleCount; i++) {
			float wheelForce = brakeForce * m_vehicleParams.axles[i].torqueFactor * m_vehicleParams.axles[i].wheels.radius;

			for (int w = 0; w < m_vehicleParams.wheelsPerAxle; w++, wheelIndex++) {
				m_pRaycastVehicle->applyEngineForce(wheelForce, wheelIndex);
			}
		}
	} else if (fabs(controls.brake) > 1e-4) {
		for (int i = 0; i < m_iWheelCount; i++) {
			m_pRaycastVehicle->applyEngineForce(0, i);
		}

		// float wheel_force_by_brake = brake_val * m_gravityLength * ( m_bodyMass + m_totalWheelMass );
		float brakeForce = controls.brake * m_pBody->GetMass();

		int wheelIndex = 0;
		for (int i = 0; i < m_vehicleParams.axleCount; i++) {
			float wheelForce = 0.5f * brakeForce * m_vehicleParams.axles[i].brakeFactor;
			for (int w = 0; w < m_vehicleParams.wheelsPerAxle; w++, wheelIndex++) {
				m_pRaycastVehicle->setBrake(wheelForce, wheelIndex);
			}
		}
	} else {
		for (int i = 0; i < m_iWheelCount; i++) {
			m_pRaycastVehicle->applyEngineForce(0, i);
		}
	}
}

CPhysicsObject *CPhysicsVehicleController::GetBody() {
	return m_pBody;
}

int CPhysicsVehicleController::GetWheelCount() {
	return m_iWheelCount;
}

IPhysicsObject *CPhysicsVehicleController::GetWheel(int index) {
	if (index >= m_iWheelCount || index < 0) return NULL;

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
		btCollisionObject *body = (btCollisionObject *)wheelInfo.m_raycastInfo.m_groundObject;
		CPhysicsObject *pObject = (CPhysicsObject *)body->getUserPointer();

		if (pContactPoint)
			ConvertPosToHL(bullContactVec, *pContactPoint);

		if (pSurfaceProps)
			*pSurfaceProps = pObject->GetMaterialIndex();

		return true;
	}

	return false;
}

void CPhysicsVehicleController::SetSpringLength(int wheelIndex, float length) {
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		Assert(0);
		return;
	}

	NOT_IMPLEMENTED
}

void CPhysicsVehicleController::SetWheelFriction(int wheelIndex, float friction) {
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		Assert(0);
		return;
	}

	NOT_IMPLEMENTED
}

void CPhysicsVehicleController::GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem) {
	memset(&debugCarSystem, 0, sizeof(debugCarSystem));
	NOT_IMPLEMENTED
}

void CPhysicsVehicleController::VehicleDataReload() {
	NOT_IMPLEMENTED
}

/******
* NEW HANDLING FUNCTIONS
******/

// force appears to be in newtons. Correct this if it's wrong.
// UNEXPOSED
void CPhysicsVehicleController::SetWheelForce(int wheelIndex, float force) {
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		Assert(0);
		return;
	}

	// convert from kg*in/s to kg*m/s
	force *= METERS_PER_INCH;
	m_pRaycastVehicle->applyEngineForce(force, wheelIndex);
}

// UNEXPOSED
void CPhysicsVehicleController::SetWheelBrake(int wheelIndex, float brakeVal) {
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		Assert(0);
		return;
	}

	// convert from kg*in/s to kg*m/s
	brakeVal *= METERS_PER_INCH;
	m_pRaycastVehicle->setBrake(brakeVal, wheelIndex);
}


// UNEXPOSED
void CPhysicsVehicleController::SetWheelSteering(int wheelIndex, float steerVal) {
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		Assert(0);
		return;
	}

	m_pRaycastVehicle->setSteeringValue(DEG2RAD(steerVal), wheelIndex);
}

/****************************
* CREATION FUNCTIONS
****************************/

IPhysicsVehicleController *CreateVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	if (!pBody) return NULL;

	CPhysicsVehicleController *pController = new CPhysicsVehicleController(pEnv, pBody, params, nVehicleType, pGameTrace);
	return pController;
}