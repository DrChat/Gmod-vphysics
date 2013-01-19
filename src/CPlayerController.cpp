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

/****************************
* CLASS CPlayerController
****************************/

CPlayerController::CPlayerController(CPhysicsObject *pObject) {
	m_pObject = pObject;
	m_handler = NULL;
	m_maxDeltaPosition = ConvertDistanceToBull(24);
	m_dampFactor = 1.0f;
	m_iTicksSinceUpdate = 0;
	AttachObject();
}

CPlayerController::~CPlayerController() {
	DetachObject();
}

void CPlayerController::Update(const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground) {
	btVector3 targetPositionBull, targetSpeedBull;

	ConvertPosToBull(position, targetPositionBull);
	ConvertPosToBull(velocity, targetSpeedBull);

	// If the object hasn't moved, abort.
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

	// AKA ensure_core_in_simulation
	//m_pObject->GetObject()->activate(true);

	m_pGround = (CPhysicsObject *)ground;

	m_iTicksSinceUpdate = 0;
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

		if (contactManifold->getNumContacts() > 0 && (obA == m_pObject->GetObject() || obB == m_pObject->GetObject())) {
			if (((CPhysicsObject *)obA->getUserPointer())->IsStatic() || ((CPhysicsObject *)obB->getUserPointer())->IsStatic())
				continue;

			return true;
		}
	}

	return false;
}

void CPlayerController::MaxSpeed(const Vector &maxVelocity) {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
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

int CPlayerController::GetShadowPosition(Vector *position, QAngle *angles) {
	btRigidBody *pObject = m_pObject->GetObject();
	btTransform transform;
	((btMassCenterMotionState *)pObject->getMotionState())->getGraphicTransform(transform);
	if (position) ConvertPosToHL(transform.getOrigin(), *position);
	if (angles) ConvertRotationToHL(transform.getBasis(), *angles);

	return m_iTicksSinceUpdate;
}

void CPlayerController::StepUp(float height) {
	btVector3 step;
	ConvertPosToBull(Vector(0, 0, height), step);
	btRigidBody *pObject = m_pObject->GetObject();
	btTransform transform;
	((btMassCenterMotionState *)pObject->getMotionState())->getWorldTransform(transform);
	transform.setOrigin(transform.getOrigin() + step);

	pObject->setWorldTransform(transform);
}

void CPlayerController::Jump() {
	// This function does absolutely nothing!
	return;
}

void CPlayerController::GetShadowVelocity(Vector *velocity) {
	if (!velocity) return;

	btRigidBody *body = m_pObject->GetObject();
	ConvertPosToHL(body->getLinearVelocity(), *velocity);
}

IPhysicsObject *CPlayerController::GetObject() {
	return m_pObject;
}

// Called in CalculateObjectStress in physics_impact_damage.cpp, may apply damage to players when hit with an object
void CPlayerController::GetLastImpulse(Vector *pOut) {
	if (!pOut) return;

	ConvertForceImpulseToHL(m_lastImpulse, *pOut);
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
	// Removed: This function was called every frame.
	// Unknown purpose, can anyone fill in what this function is used for?
	//NOT_IMPLEMENTED
	return false;
}

void CPlayerController::Tick(float deltaTime) {
	if (!m_enable)
		return;

	m_iTicksSinceUpdate++;

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

	m_lastImpulse = speed;
}

void CPlayerController::AttachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	m_saveRot = body->getAngularFactor();
	body->setAngularFactor(0);

	m_pObject->AddCallbackFlags(CALLBACK_IS_PLAYER_CONTROLLER);
	//body->setCollisionFlags(body->getCollisionFlags() | btRigidBody::CF_KINEMATIC_OBJECT);

	body->setActivationState(DISABLE_DEACTIVATION);
}

void CPlayerController::DetachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	body->setAngularFactor(m_saveRot);
	body->setActivationState(ACTIVE_TAG);

	m_pObject->RemoveCallbackFlags(CALLBACK_IS_PLAYER_CONTROLLER);
	//body->setCollisionFlags(body->getCollisionFlags() & ~(btRigidBody::CF_KINEMATIC_OBJECT));

	m_pObject = NULL;
}

int CPlayerController::TryTeleportObject() {
	if (m_handler) {
		Vector hlPosition;
		ConvertPosToHL(m_targetPosition, hlPosition);
		if (!m_handler->ShouldMoveTo(m_pObject, hlPosition)) return 0;
	}

	btRigidBody *body = m_pObject->GetObject();
	btTransform transform;
	((btMassCenterMotionState *)body->getMotionState())->getGraphicTransform(transform);
	transform.setOrigin(m_targetPosition);
	((btMassCenterMotionState *)body->getMotionState())->setGraphicTransform(transform);

	btTransform finaltrans;
	((btMassCenterMotionState *)body->getMotionState())->getWorldTransform(finaltrans);

	body->setWorldTransform(finaltrans);
	return 1;
}