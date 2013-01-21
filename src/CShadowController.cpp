#include "StdAfx.h"

#include "CPhysicsEnvironment.h"
#include "CShadowController.h"
#include "CPlayerController.h"
#include "CPhysicsObject.h"
#include "CPhysicsSurfaceProps.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

// INFO ON QUATERNIONS
// -q and q represent the same orientation!

inline static void QuaternionDiff(const btQuaternion &p, const btQuaternion &q, btQuaternion &qt) {
	qt = q.inverse() * p;
	qt.normalize();
}

static ConVar cvar_spewshadowdebuginfo("vphysics_spewshadowcontrollerdebuginfo", "0", FCVAR_CHEAT | FCVAR_ARCHIVE);
static ConVar cvar_shadowfix("vphysics_useshadowfix", "1", FCVAR_ARCHIVE);
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

			((btMassCenterMotionState *)object->getMotionState())->setGraphicTransform(transform);
			btTransform finalTrans;
			((btMassCenterMotionState *)object->getMotionState())->getWorldTransform(finalTrans);
			object->setWorldTransform(finalTrans);
		}
	}

	float invDt = SAFE_DIVIDE(1.0f, dt);
	btVector3 speed = object->getLinearVelocity();
	ComputeController(speed, delta_position, params.maxSpeed, fraction * invDt, params.dampFactor);
	object->setLinearVelocity(speed);

	params.lastPosition = posbull + speed * dt;

	// Set the angles
	btVector3 deltaAngles;
	btQuaternion deltaRotation;
	QuaternionDiff(params.targetRotation, transform.getRotation(), deltaRotation);

	//*
	// When the target rotation angle is b/t 1 and 5, screwups start occurring.
	btVector3 axis = deltaRotation.getAxis();
	btScalar angle = deltaRotation.getAngle();
	axis.normalize();

	// We don't want a super long rotation delta.
	if (angle > M_PI) {
		angle -= 2 * M_PI;
	}

	deltaAngles = axis * angle;
	//*/

	btVector3 rot_speed = object->getAngularVelocity();
	// DEBUG
	if (cvar_spewshadowdebuginfo.GetBool()) {
		btQuaternion transquat = transform.getRotation();
		Msg("Bull Transform Rotation Quat: %f %f %f %f\n", transquat.getAxis().x(), transquat.getAxis().y(), transquat.getAxis().z(), transquat.getAngle());
		Msg("Target Rotation Quat: %f %f %f %f\n", params.targetRotation.getAxis().x(), params.targetRotation.getAxis().y(), params.targetRotation.getAxis().z(), params.targetRotation.getAngle());
		Msg("Delta Rotation Quat: %f %f %f %f\n", deltaRotation.getAxis().x(), deltaRotation.getAxis().y(), deltaRotation.getAxis().z(), deltaRotation.getAngle());
		Msg("-- Delta Angles: %f %f %f --\n\n", deltaAngles.x(), deltaAngles.y(), deltaAngles.z());

		CPhysicsEnvironment *pEnv = ((CPhysicsObject *)object->getUserPointer())->GetVPhysicsEnvironment();
		if (pEnv) {
			IVPhysicsDebugOverlay *pOverlay = pEnv->GetDebugOverlay();
			if (pOverlay) {
				Vector origin;
				ConvertPosToHL(params.targetRotation.getAxis(), origin);
				pOverlay->AddTextOverlay(origin, 0, -1, "%f %f %f", origin.x, origin.y, origin.z);
				pOverlay->AddTextOverlay(origin, 1, -1, "%f", params.targetRotation.getAngle());
				pOverlay->AddLineOverlay(Vector(0, 0, 0), origin, 255, 0, 0, false, 0);
				pOverlay->AddTextOverlay(Vector(0, 0, 0), 0, 0, "Origin");
			}
		}
	}
	// END DEBUG

	ComputeController(rot_speed, deltaAngles, params.maxAngular, fraction * invDt, params.dampFactor);
	object->setAngularVelocity(rot_speed);
	
	// HACK: Replace this soon!
	if (cvar_shadowfix.GetBool()) {
		object->setAngularVelocity(btVector3(0, 0, 0));
		btTransform targTrans;
		targTrans.setOrigin(params.targetPosition);
		targTrans.setRotation(params.targetRotation);
		object->setWorldTransform(targTrans);
	}

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

/***************************
* CLASS CShadowController
***************************/

CShadowController::CShadowController(CPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	m_pObject = pObject;
	m_shadow.dampFactor = 1.0f;
	m_shadow.teleportDistance = 0;
	m_shadow.targetPosition.setZero();
	memset(&m_shadow.targetRotation, 0, sizeof(btQuaternion)); // Shit fucking access violation randomly caused by below line in release builds
	//m_shadow.targetRotation = btQuaternion(0, 0, 0, 1);
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
		// TODO: Figure out the intended behavior and set accordingly. We may just want to smoothly move the object
		// to point B without applying velocities to achieve this. IVP shadows don't respond to collisions, and
		// neither should we.
		//ComputeShadowControllerBull(m_pObject->GetObject(), m_shadow, m_secondsToArrival, deltaTime);
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

	//m_pObject->Wake();
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
	m_shadow.teleportDistance = ConvertDistanceToBull(teleportDistance);
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

// NPCs call this
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

// FIXME: What do we return? Ticks since last update?
float CShadowController::GetTargetPosition(Vector *pPositionOut, QAngle *pAnglesOut) {
	if (!pPositionOut && !pAnglesOut) return 0;

	if (pPositionOut)
		ConvertPosToHL(m_shadow.targetPosition, *pPositionOut);

	if (pAnglesOut)
		ConvertRotationToHL(m_shadow.targetRotation, *pAnglesOut);

	return 0;
}

float CShadowController::GetTeleportDistance() {
	return ConvertDistanceToHL(m_shadow.teleportDistance);
}

void CShadowController::GetMaxSpeed(float *pMaxSpeedOut, float *pMaxAngularSpeedOut) {
	if (!pMaxSpeedOut && !pMaxAngularSpeedOut) return;
	NOT_IMPLEMENTED
}

void CShadowController::AttachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	m_savedMass = SAFE_DIVIDE(1, body->getInvMass());
	m_savedMaterialIndex = m_pObject->GetMaterialIndex();

	m_pObject->SetMaterialIndex(MATERIAL_INDEX_SHADOW);

	if (!m_allowPhysicsMovement) {
		m_pObject->SetMass(0);
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

	body->setCollisionFlags(body->getCollisionFlags() & ~(btCollisionObject::CF_KINEMATIC_OBJECT));
	body->setActivationState(ACTIVE_TAG);
}
