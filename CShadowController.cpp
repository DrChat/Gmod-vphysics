#include "StdAfx.h"

#include "CShadowController.h"
#include "CPhysicsObject.h"
#include "convert.h"

CShadowController::CShadowController(CPhysicsObject* pObject, bool allowTranslation, bool allowRotation) {
	m_pObject = pObject;
	btRigidBody* body = pObject->GetObject();
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(DISABLE_DEACTIVATION);
}

CShadowController::~CShadowController() {

}

void CShadowController::Update(const Vector &position, const QAngle &angles, float timeOffset) {
	btVector3 btvec;
	btMatrix3x3 matrix;

	ConvertPosToBull(position, btvec);
	ConvertRotationToBull(angles, matrix);

	btTransform transform(matrix, btvec);
	((btMassCenterMotionState*)m_pObject->GetObject()->getMotionState())->setGraphicTransform(transform);
}

void CShadowController::MaxSpeed(float maxSpeed, float maxAngularSpeed) {
	NOT_IMPLEMENTED;
}

void CShadowController::StepUp(float height) {
	NOT_IMPLEMENTED;
}

void CShadowController::SetTeleportDistance(float teleportDistance) {
	NOT_IMPLEMENTED;
}

bool CShadowController::AllowsTranslation() {
	NOT_IMPLEMENTED;
	return false;
}

bool CShadowController::AllowsRotation() {
	NOT_IMPLEMENTED;
	return false;
}

void CShadowController::SetPhysicallyControlled(bool isPhysicallyControlled) {
	NOT_IMPLEMENTED;
}

bool CShadowController::IsPhysicallyControlled() {
	NOT_IMPLEMENTED;
	return false;
}

void CShadowController::GetLastImpulse(Vector* pOut) {
	NOT_IMPLEMENTED;
}

void CShadowController::UseShadowMaterial(bool bUseShadowMaterial) {
	NOT_IMPLEMENTED;
}

void CShadowController::ObjectMaterialChanged(int materialIndex) {
	NOT_IMPLEMENTED;
}

float CShadowController::GetTargetPosition(Vector* pPositionOut, QAngle* pAnglesOut) {
	NOT_IMPLEMENTED;
	return 0;
}

float CShadowController::GetTeleportDistance() {
	NOT_IMPLEMENTED;
	return 0;
}

void CShadowController::GetMaxSpeed(float* pMaxSpeedOut, float* pMaxAngularSpeedOut) {
	NOT_IMPLEMENTED;
}
