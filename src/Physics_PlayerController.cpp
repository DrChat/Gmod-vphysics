#include "StdAfx.h"

#include "Physics_PlayerController.h"
#include "Physics_Object.h"
#include "Physics_Environment.h"
#include "convert.h"
#include "math.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

void ComputeController(btVector3 &currentSpeed, const btVector3 &delta, const btVector3 &maxSpeed, float scaleDelta, float damping) {
	// Timestep scale
	btVector3 acceleration = delta * scaleDelta;

	if (currentSpeed.fuzzyZero() && !currentSpeed.isZero()) {
		currentSpeed.setZero();
	}
	acceleration += currentSpeed * -damping;

	// Clamp the acceleration to max speed
	for(int i = 2; i >= 0; i--) {
		if (fabs(acceleration[i]) < maxSpeed[i]) continue;
		acceleration[i] = (acceleration[i] < 0) ? -maxSpeed[i] : maxSpeed[i];
	}

	currentSpeed += acceleration;
}

/****************************
* CLASS CPlayerController
****************************/

CPlayerController::CPlayerController(CPhysicsEnvironment *pEnv, CPhysicsObject *pObject) {
	m_pObject = pObject;
	m_pEnv = pEnv;
	m_handler = NULL;
	m_maxDeltaPosition = ConvertDistanceToBull(24);
	m_dampFactor = 1.0f;
	m_ticksSinceUpdate = 0;
	m_lastImpulse = btVector3(0, 0, 0);
	m_linVelocity = btVector3(0, 0, 0);

	AttachObject();
}

CPlayerController::~CPlayerController() {
	DetachObject();
}

void CPlayerController::Update(const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground) {
	btVector3 bullTargetPosition, bullMaxVelocity;

	ConvertPosToBull(position, bullTargetPosition);
	ConvertPosToBull(velocity, bullMaxVelocity);

	// If the targets haven't changed, abort.
	if (bullMaxVelocity.distance2(m_maxVelocity) < FLT_EPSILON && bullTargetPosition.distance2(m_targetPosition) < FLT_EPSILON) {
		return;
	}

	m_targetPosition = bullTargetPosition;
	m_maxVelocity = bullMaxVelocity;

	m_enable = true;

	// FYI: The onground stuff includes any props we may be standing on as well as the world.
	// The ground is valid only if it's significantly heavier than our object ("Rideable physics" > our mass * 2)
	m_onground = onground;

	if (velocity.LengthSqr() <= 0.001f) {
		m_enable = false;
		ground = NULL;
	} else {
		MaxSpeed(velocity);
	}

	m_secondsToArrival = secondsToArrival;

	m_pGround = (CPhysicsObject *)ground;

	m_ticksSinceUpdate = 0;
}

void CPlayerController::SetEventHandler(IPhysicsPlayerControllerEvent *handler) {
	m_handler = handler;
}

bool CPlayerController::IsInContact() {
	CPhysicsEnvironment *pEnv = m_pObject->GetVPhysicsEnvironment();

	int numManifolds = pEnv->GetBulletEnvironment()->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold *contactManifold = pEnv->GetBulletEnvironment()->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject *obA = contactManifold->getBody0();
		const btCollisionObject *obB = contactManifold->getBody1();
		CPhysicsObject *pPhysUs = NULL;
		CPhysicsObject *pPhysOther = NULL;

		if (contactManifold->getNumContacts() > 0 && (obA == m_pObject->GetObject() || obB == m_pObject->GetObject())) {
			if (obA == m_pObject->GetObject()) {
				pPhysUs = (CPhysicsObject *)obA->getUserPointer();
				pPhysOther = (CPhysicsObject *)obB->getUserPointer();
			} else if (obB == m_pObject->GetObject()) {
				pPhysUs = (CPhysicsObject *)obB->getUserPointer();
				pPhysOther = (CPhysicsObject *)obA->getUserPointer();
			}

			if (pPhysOther->IsStatic() || pPhysOther->GetCallbackFlags() & CALLBACK_SHADOW_COLLISION)
				continue;

			return true;
		}
	}

	return false;
}

void CPlayerController::MaxSpeed(const Vector &maxVelocity) {
	btVector3 bullVel;
	ConvertPosToBull(maxVelocity, bullVel);
	btVector3 available = bullVel;

	float length = bullVel.length();
	bullVel.normalize();

	float dot = bullVel.dot(m_linVelocity);
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

int CPlayerController::GetShadowPosition(Vector *position, QAngle *angles) {
	btRigidBody *pObject = m_pObject->GetObject();
	btTransform transform;
	((btMassCenterMotionState *)pObject->getMotionState())->getGraphicTransform(transform);
	if (position) ConvertPosToHL(transform.getOrigin(), *position);
	if (angles) ConvertRotationToHL(transform.getBasis(), *angles);

	// Yep. We're returning a variable totally unrelated to the shadow's position.
	return m_ticksSinceUpdate;
}

void CPlayerController::StepUp(float height) {
	btVector3 step;
	ConvertPosToBull(Vector(0, 0, height), step);

	btRigidBody *pObject = m_pObject->GetObject();
	btTransform transform = pObject->getWorldTransform();
	transform.setOrigin(transform.getOrigin() + step);

	pObject->getMotionState()->setWorldTransform(transform);
}

void CPlayerController::Jump() {
	// This function does absolutely nothing!
	return;
}

void CPlayerController::GetShadowVelocity(Vector *velocity) {
	if (!velocity) return;

	//btRigidBody *body = m_pObject->GetObject();
	//ConvertPosToHL(body->getLinearVelocity(), *velocity);

	ConvertPosToHL(m_linVelocity, *velocity);
}

IPhysicsObject *CPlayerController::GetObject() {
	return m_pObject;
}

void CPlayerController::GetLastImpulse(Vector *pOut) {
	if (!pOut) return;

	ConvertForceImpulseToHL(m_lastImpulse, *pOut);
}

void CPlayerController::SetPushMassLimit(float maxPushMass) {
	m_pushMassLimit = maxPushMass;
}

void CPlayerController::SetPushSpeedLimit(float maxPushSpeed) {
	m_pushSpeedLimit = maxPushSpeed;
}

float CPlayerController::GetPushMassLimit() {
	return m_pushMassLimit;
}

float CPlayerController::GetPushSpeedLimit() {
	return m_pushSpeedLimit;
}

bool CPlayerController::WasFrozen() {
	// Removed: This function was called every frame.
	// Unknown purpose, can anyone fill in what this function is used for?
	// Appears that if we were frozen, the game will try and update our position to the player's current position.
	//NOT_IMPLEMENTED
	return false;
}

void CPlayerController::Tick(float deltaTime) {
	if (!m_enable && !IsInContact())
		return;

	m_ticksSinceUpdate++;

	btRigidBody *body = m_pObject->GetObject();
	btMassCenterMotionState *motionState = (btMassCenterMotionState *)body->getMotionState();

	// Don't let the player controller travel too far away from the target position.
	btTransform transform;
	motionState->getGraphicTransform(transform);
	btVector3 delta_position = m_targetPosition - transform.getOrigin();

	btScalar qdist = delta_position.length2();
	if (qdist > m_maxDeltaPosition * m_maxDeltaPosition && TryTeleportObject()) {
		return;
	}

	m_linVelocity.setZero();
	CalculateVelocity(deltaTime);

	// Integrate the velocity into our world transform
	btVector3 deltaPos = m_linVelocity * deltaTime;

	if (!deltaPos.fuzzyZero()) {
		btTransform trans;
		motionState->getGraphicTransform(trans);
		trans.setOrigin(trans.getOrigin() + deltaPos);
		motionState->setGraphicTransform(trans);
	}
}

void CPlayerController::CalculateVelocity(float dt) {
	btRigidBody *body = m_pObject->GetObject();

	// Compute controller speed
	// When we fling off, ComputeController gets a HUGE scale

	m_secondsToArrival -= dt;
	if (m_secondsToArrival < 0) m_secondsToArrival = 0;

	float psiScale = m_pEnv->GetInvPSIScale();

	btTransform transform;
	((btMassCenterMotionState *)body->getMotionState())->getGraphicTransform(transform);
	btVector3 deltaPos = m_targetPosition - transform.getOrigin();

	ComputeController(m_linVelocity, deltaPos, m_maxSpeed, SAFE_DIVIDE(psiScale, dt), m_dampFactor);

	if (abs(m_linVelocity.m_floats[0]) > 50 || abs(m_linVelocity.m_floats[1]) > 50 || abs(m_linVelocity.m_floats[2]) > 50) {
		Msg("uhoh psiScale %f\n", psiScale);
		Msg("delta %f %f %f\n", deltaPos.x(), deltaPos.y(), deltaPos.z());
	}

	// Apply gravity velocity for stepping.
	if (m_onground) {
		btVector3 gravVel = body->getGravity() * dt;
		m_linVelocity += gravVel;
	}
}

void CPlayerController::AttachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	m_saveRot = body->getAngularFactor();
	body->setAngularFactor(0);

	m_pObject->AddCallbackFlags(CALLBACK_IS_PLAYER_CONTROLLER);
	body->setCollisionFlags(body->getCollisionFlags() | btRigidBody::CF_KINEMATIC_OBJECT);

	body->setActivationState(DISABLE_DEACTIVATION);
}

void CPlayerController::DetachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	body->setAngularFactor(m_saveRot);
	body->setActivationState(ACTIVE_TAG);

	m_pObject->RemoveCallbackFlags(CALLBACK_IS_PLAYER_CONTROLLER);
	body->setCollisionFlags(body->getCollisionFlags() & ~(btRigidBody::CF_KINEMATIC_OBJECT));

	m_pObject = NULL;
}

bool CPlayerController::TryTeleportObject() {
	if (m_handler) {
		Vector hlPosition;
		ConvertPosToHL(m_targetPosition, hlPosition);
		if (!m_handler->ShouldMoveTo(m_pObject, hlPosition)) return false;
	}

	btRigidBody *body = m_pObject->GetObject();

	btTransform trans = body->getWorldTransform();
	trans.setOrigin(m_targetPosition);

	body->setWorldTransform(trans  * ((btMassCenterMotionState *)body->getMotionState())->m_centerOfMassOffset);
	((btMassCenterMotionState *)body->getMotionState())->setGraphicTransform(trans);

	return true;
}

/***********************
* CREATION FUNCTIONS
***********************/

CPlayerController *CreatePlayerController(CPhysicsEnvironment *pEnv, IPhysicsObject *pObject) {
	if (!pObject) return NULL;

	return new CPlayerController(pEnv, (CPhysicsObject *)pObject);
}