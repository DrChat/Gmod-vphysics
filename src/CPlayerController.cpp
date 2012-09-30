#include "StdAfx.h"

#include "CPlayerController.h"
#include "CPhysicsObject.h"
#include "CPhysicsEnvironment.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

void ComputeController(btVector3 &currentSpeed, const btVector3 &delta, const btVector3 &maxSpeed, float scaleDelta, float damping) {
	// Timestep scale
	btVector3 acceleration = delta * scaleDelta;

	if (currentSpeed.length2() < 1e-6) {
		currentSpeed.setZero();
	}
	acceleration += currentSpeed * -damping;

	// Clamp the acceleration to max speed
	for(int i=2; i>=0; i--) {
		if (fabs(acceleration[i]) < maxSpeed[i]) continue;
		acceleration[i] = (acceleration[i] < 0) ? -maxSpeed[i] : maxSpeed[i];
	}

	currentSpeed += acceleration;
}

CPlayerController::CPlayerController(CPhysicsObject *pObject) {
	m_pObject = pObject;
	m_handler = NULL;
	m_maxDeltaPosition = HL2BULL(24); // ConvertDistanceToBull(24);
	m_dampFactor = 1.0f;
	AttachObject();
}

CPlayerController::~CPlayerController() {
	DetachObject();
}

void CPlayerController::Update(const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground) {
	btVector3 targetPositionBull, targetSpeedBull;

	ConvertPosToBull(position, targetPositionBull);
	ConvertPosToBull(velocity, targetSpeedBull);

	if (targetSpeedBull.distance2(m_currentSpeed) < 1e-6) {
		if (targetPositionBull.distance2(m_targetPosition) < 1e-6) {
			return;
		}
	}

	m_targetPosition = targetPositionBull;
	m_currentSpeed = targetSpeedBull;

	m_enable = true;
	m_onground = onground;

	if (velocity.LengthSqr() <= 0.1f) {
		m_enable = false;
		ground = NULL;
	} else {
		MaxSpeed(velocity);
	}
}

void CPlayerController::SetEventHandler(IPhysicsPlayerControllerEvent* handler) {
	m_handler = handler;
}

bool CPlayerController::IsInContact() {
	//NOT_IMPLEMENTED;
	return false;
}

void CPlayerController::MaxSpeed(const Vector& maxVelocity) {
	btRigidBody* body = btRigidBody::upcast(m_pObject->GetObject());
	btVector3 bullVel;
	ConvertPosToBull(maxVelocity, bullVel);
	btVector3 available = bullVel;

	float length = bullVel.length();
	bullVel.normalize();

	float dot = bullVel.dot(body->getLinearVelocity());
	if (dot > 0) {
		bullVel *= dot * length;
		available -= bullVel;
	}

	m_maxSpeed = available.absolute();
}

void CPlayerController::SetObject(IPhysicsObject *pObject) {
	if (pObject == m_pObject)
		return;

	DetachObject();
	m_pObject = (CPhysicsObject *)pObject;
	AttachObject();
}

int CPlayerController::GetShadowPosition( Vector *position, QAngle *angles ) {
	btRigidBody *pObject = m_pObject->GetObject();
	btTransform transform;
	((btMassCenterMotionState*)pObject->getMotionState())->getGraphicTransform(transform);
	if (position) ConvertPosToHL(transform.getOrigin(), *position);
	if (angles) ConvertRotationToHL(transform.getBasis(), *angles);
	// FIXME: what is this?
	// Andrew; this needs to return the amount of ticks since the last Update()
	return 0;
}

void CPlayerController::StepUp(float height) {
	btVector3 step;
	ConvertPosToBull(Vector(0, 0, height), step);
	btRigidBody *pObject = m_pObject->GetObject();
	btTransform transform;
	((btMassCenterMotionState*)pObject->getMotionState())->getGraphicTransform(transform);
	transform.setOrigin(transform.getOrigin()+step);
}

void CPlayerController::Jump() {
	return;
}

void CPlayerController::GetShadowVelocity(Vector *velocity) {
	btRigidBody *body = m_pObject->GetObject();
	ConvertPosToHL(body->getLinearVelocity(), *velocity);
	NOT_IMPLEMENTED;
}

IPhysicsObject *CPlayerController::GetObject() {
	return m_pObject;
}

// Called with NPCs
void CPlayerController::GetLastImpulse(Vector *pOut) {
	NOT_IMPLEMENTED;
}

void CPlayerController::SetPushMassLimit(float maxPushMass) {
	m_PushMassLimit = maxPushMass;
}

void CPlayerController::SetPushSpeedLimit(float maxPushSpeed) {
	m_PushSpeedLimit = maxPushSpeed;
}

float CPlayerController::GetPushMassLimit() {
	return m_PushMassLimit;
}

float CPlayerController::GetPushSpeedLimit() {
	return m_PushSpeedLimit;
}

bool CPlayerController::WasFrozen() {
	NOT_IMPLEMENTED;
	return false;
}

// TODO: Players walking ontop of physical objects causes players
// to fly off. About 90% reproducible in gm_construct white room
// when walking
// Goes in -x +y direction when near front doors of white room
// At back, -x -y
void CPlayerController::Tick(float deltaTime) {
	if (!m_enable)
		return;

	btRigidBody *body = m_pObject->GetObject();
	CPhysicsEnvironment *pEnv = m_pObject->GetVPhysicsEnvironment();
	btDynamicsWorld *world = pEnv->GetBulletEnvironment();

	float psiScale = pEnv->GetInvPSIScale();
	if (!psiScale) return;

	btTransform transform;
	((btMassCenterMotionState *)body->getMotionState())->getGraphicTransform(transform);
	btVector3 cur_pos = transform.getOrigin();

	btVector3 delta_position = m_targetPosition - cur_pos;

	//FIXME: figure out what shift_core_f_object is
	// shift_core_f_object is a floating point vector3
	// shift core from object (displacement of core from object)?
	// IVP docs appear to confirm _f_ means from
	// Source: (Note to the syntax used in the Ipion engine: ”m_world_f_object” means: Matrix which transforms into world space from object space.) 

	btScalar qdist = delta_position.length2();
	if (qdist > m_maxDeltaPosition * m_maxDeltaPosition && TryTeleportObject()) {
		return;
	}

	if (!m_onground) {
		btVector3 pgrav = world->getGravity();
		btVector3 gravSpeed = pgrav * deltaTime;
		body->setLinearVelocity(body->getLinearVelocity() - gravSpeed);
	}

	btVector3 speed = body->getLinearVelocity();
	ComputeController(speed, delta_position, m_maxSpeed, psiScale / deltaTime, m_dampFactor);
	body->setLinearVelocity(speed);
}

void CPlayerController::AttachObject() {
	btRigidBody* body = btRigidBody::upcast(m_pObject->GetObject());
	m_saveRot = body->getAngularDamping();
	body->setDamping(body->getLinearDamping(), 100);

	body->setActivationState(DISABLE_DEACTIVATION);
}

void CPlayerController::DetachObject() {
	btRigidBody* body = btRigidBody::upcast(m_pObject->GetObject());
	body->setDamping(body->getLinearDamping(), m_saveRot);
	m_pObject = NULL;

	body->setActivationState(ACTIVE_TAG);
}

int CPlayerController::TryTeleportObject() {
	if (m_handler) {
		Vector hlPosition;
		ConvertPosToHL(m_targetPosition, hlPosition);
		if (!m_handler->ShouldMoveTo(m_pObject, hlPosition)) return 0;
	}

	btRigidBody* body = m_pObject->GetObject();
	btTransform transform;
	((btMassCenterMotionState*)body->getMotionState())->getGraphicTransform(transform);

	transform.setOrigin(m_targetPosition);
	((btMassCenterMotionState*)body->getMotionState())->setGraphicTransform(transform);
	return 1;
}