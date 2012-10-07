#include "StdAfx.h"

#include "CShadowController.h"
#include "CPlayerController.h"
#include "CPhysicsObject.h"
#include "CPhysicsSurfaceProps.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

void QuaternionDiff(const btQuaternion &p, const btQuaternion &q, btQuaternion &qt) {
	qt = q.inverse() * p;
	qt.normalize();
}

float ComputeShadowControllerBull(btRigidBody *object, shadowcontrol_params_t &params, float secondsToArrival, float dt) {
	float fraction = 1.0;
	if (secondsToArrival > 0) {
		fraction *= dt / secondsToArrival;
		if (fraction > 1) fraction = 1;
	}

	secondsToArrival -= dt;
	if (secondsToArrival < 0) secondsToArrival = 0;

	if (fraction <= 0) return secondsToArrival;

	btTransform transform;
	((btMassCenterMotionState *)object->getMotionState())->getGraphicTransform(transform);

	btVector3 posbull = transform.getOrigin();
	btVector3 delta_position = params.targetPosition - posbull;

	if (params.teleportDistance > 0) {
		btScalar qdist;
		if (!params.lastPosition.isZero()) {
			btVector3 tmpDelta = posbull - params.lastPosition;
			qdist = tmpDelta.length2();
		} else {
			qdist = delta_position.length2();
		}

		if (qdist > params.teleportDistance * params.teleportDistance) {
			transform.setOrigin(params.targetPosition);
			transform.setRotation(params.targetRotation);
		}
	}

	float invDt = 1.0f / dt;
	btVector3 speed = object->getLinearVelocity();
	ComputeController(speed, delta_position, params.maxSpeed, fraction * invDt, params.dampFactor);
	object->setLinearVelocity(speed);

	params.lastPosition = posbull + speed * dt;

	// Set the angles
	btVector3 deltaAngles;
	btQuaternion deltaRotation; 
	QuaternionDiff(params.targetRotation, transform.getRotation(), deltaRotation);

	btVector3 axis = deltaRotation.getAxis();
	float angle = deltaRotation.getAngle();
	axis.normalize();

	deltaAngles.setX(axis.x() * angle);
	deltaAngles.setY(axis.y() * angle);
	deltaAngles.setZ(axis.z() * angle);

	btVector3 rot_speed = object->getAngularVelocity();
	ComputeController(rot_speed, deltaAngles, params.maxAngular, fraction * invDt, params.dampFactor);
	object->setAngularVelocity(rot_speed);

	//object->setAngularVelocity(btVector3(0, 0, 0));

	return secondsToArrival;
}

void ConvertShadowControllerToBull(const hlshadowcontrol_params_t &in, shadowcontrol_params_t &out) {
	ConvertPosToBull(in.targetPosition, out.targetPosition);
	ConvertRotationToBull(in.targetRotation, out.targetRotation);
	out.teleportDistance = ConvertDistanceToBull(in.teleportDistance);

	ConvertForceImpulseToBull(in.maxSpeed, out.maxSpeed);
	out.maxSpeed = out.maxSpeed.absolute();
	ConvertAngularImpulseToBull(in.maxAngular, out.maxAngular);
	out.maxAngular = out.maxAngular.absolute();
	out.dampFactor = in.dampFactor;
}

float ComputeShadowControllerHL(CPhysicsObject *pObject, const hlshadowcontrol_params_t &params, float secondsToArrival, float dt) {
	shadowcontrol_params_t bullParams;
	ConvertShadowControllerToBull(params, bullParams);
	return ComputeShadowControllerBull(pObject->GetObject(), bullParams, secondsToArrival, dt);
}

static bool IsEqual(const btQuaternion &pt0, const btQuaternion &pt1) {
	float delta = fabs(pt0.x() - pt1.x());
	delta += fabs(pt0.y() - pt1.y());
	delta += fabs(pt0.z() - pt1.z());
	delta += fabs(pt0.w() - pt1.w());
	return delta < 1e-8f;
}

static bool IsEqual(const btVector3 &pt0, const btVector3 &pt1) {
	return pt0.distance2(pt1) < 1e-8f;
}

CShadowController::CShadowController(CPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	m_pObject = pObject;
	m_shadow.dampFactor = 1.0f;
	m_shadow.teleportDistance = 0;
	m_bPhysicallyControlled = false;

	m_allowPhysicsMovement = allowTranslation;
	m_allowPhysicsRotation = allowRotation;
	AttachObject();
}

CShadowController::~CShadowController() {
	DetachObject();
}

// UNEXPOSED
void CShadowController::Tick(float deltaTime) {
	if (m_enable) {
		ComputeShadowControllerBull(m_pObject->GetObject(), m_shadow, m_secondsToArrival, deltaTime);
		m_secondsToArrival -= deltaTime;
		if (m_secondsToArrival < 0) m_secondsToArrival = 0;
	} else {
		m_shadow.lastPosition.setZero();
	}
}

void CShadowController::Update(const Vector &position, const QAngle &angles, float timeOffset) {
	btVector3 targetPosition = m_shadow.targetPosition;
	btQuaternion targetRotation = m_shadow.targetRotation;

	ConvertPosToBull(position, m_shadow.targetPosition);
	ConvertRotationToBull(angles, m_shadow.targetRotation);
	m_secondsToArrival = timeOffset < 0 ? 0 : timeOffset;

	m_enable = true;

	if (IsEqual(targetPosition, m_shadow.targetPosition) && IsEqual(targetRotation, m_shadow.targetRotation)) return;

	m_pObject->Wake();
}

void CShadowController::MaxSpeed(float maxSpeed, float maxAngularSpeed) {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());

	btVector3 bullSpeed;
	ConvertPosToBull(maxSpeed, bullSpeed);
	btVector3 available = bullSpeed;

	m_currentSpeed = bullSpeed;

	float length = bullSpeed.length();
	bullSpeed.normalize();

	float dot = bullSpeed.dot(body->getLinearVelocity());
	if (dot > 0) {
		bullSpeed *= dot * length;
		available -= bullSpeed;
	}
	m_shadow.maxSpeed = available.absolute();

	btVector3 bullAngular;
	ConvertAngularImpulseToBull(maxAngularSpeed, bullAngular);
	btVector3 availableAngular;

	float lengthAngular = bullAngular.length();
	bullAngular.normalize();

	float dotAngular = bullAngular.dot(body->getAngularVelocity());
	if (dotAngular > 0) {
		bullAngular *= dotAngular * lengthAngular;
		availableAngular -= bullAngular;
	}
	m_shadow.maxAngular = availableAngular.absolute();
}

void CShadowController::StepUp(float height) {
	NOT_IMPLEMENTED
}

void CShadowController::SetTeleportDistance(float teleportDistance) {
	NOT_IMPLEMENTED
}

bool CShadowController::AllowsTranslation() {
	return m_allowPhysicsMovement;
}

bool CShadowController::AllowsRotation() {
	return m_allowPhysicsRotation;
}

void CShadowController::SetPhysicallyControlled(bool isPhysicallyControlled) {
	m_bPhysicallyControlled = isPhysicallyControlled;
}

bool CShadowController::IsPhysicallyControlled() {
	return m_bPhysicallyControlled;
}

void CShadowController::GetLastImpulse(Vector *pOut) {
	if (!pOut) return;

	NOT_IMPLEMENTED
	*pOut = Vector(0,0,0);
}

void CShadowController::UseShadowMaterial(bool bUseShadowMaterial) {
	NOT_IMPLEMENTED
}

void CShadowController::ObjectMaterialChanged(int materialIndex) {
	// (assumed)
	// if (m_bUseShadowMaterial) {
	//		m_iObjectMaterial = materialIndex
	// }

	NOT_IMPLEMENTED
}

float CShadowController::GetTargetPosition(Vector *pPositionOut, QAngle *pAnglesOut) {
	if (!pPositionOut && !pAnglesOut) return 0;

	NOT_IMPLEMENTED
	return 0;
}

float CShadowController::GetTeleportDistance() {
	NOT_IMPLEMENTED
	return 0;
}

void CShadowController::GetMaxSpeed(float *pMaxSpeedOut, float *pMaxAngularSpeedOut) {
	NOT_IMPLEMENTED
}

void CShadowController::AttachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	m_savedMass = SAFE_DIVIDE(1, body->getInvMass());
	m_savedMaterialIndex = m_pObject->GetMaterialIndex();

	m_pObject->SetMaterialIndex(MATERIAL_INDEX_SHADOW);

	if ( !m_allowPhysicsMovement ) {
		m_pObject->SetMass(1e6f);
		m_pObject->EnableGravity(false);
	}

	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(DISABLE_DEACTIVATION);
}

void CShadowController::DetachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	btVector3 btvec = body->getInvInertiaDiagLocal();
	btvec.setX(SAFE_DIVIDE(1.0, btvec.x()));
	btvec.setY(SAFE_DIVIDE(1.0, btvec.y()));
	btvec.setZ(SAFE_DIVIDE(1.0, btvec.z()));
	body->setMassProps(m_savedMass, btvec);
	m_pObject->SetMaterialIndex(m_savedMaterialIndex);

	body->setCollisionFlags(body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(ACTIVE_TAG);
}
