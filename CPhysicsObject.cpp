#include "StdAfx.h"

#include "CPhysicsObject.h"
#include "CPhysicsEnvironment.h"
#include "CPhysicsCollision.h"
#include "CPhysicsFrictionSnapshot.h"
#include "CShadowController.h"
#include "convert.h"

CPhysicsObject *CreatePhysicsObject(CPhysicsEnvironment *pEnvironment, const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle& angles, objectparams_t *pParams, bool isStatic) {
	btCompoundShape* shape = (btCompoundShape*)pCollisionModel;
	
	btTransform transform;
	transform.setIdentity();
	btQuaternion quat;
	ConvertRotationToBull(angles, quat);
	transform.setRotation(quat);
	btVector3 vector;
	ConvertPosToBull(position, vector);
	transform.setOrigin(vector);

	float mass = pParams->mass;
	if (isStatic) mass = 0;

	btMotionState* motionstate = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo info(mass,motionstate,shape);
	info.m_linearDamping = pParams->damping;
	btRigidBody* body = new btRigidBody(info);

	pEnvironment->GetBulletEnvironment()->addRigidBody(body);

	CPhysicsObject *pObject = new CPhysicsObject();
	pObject->Init(pEnvironment, body);
	pObject->SetGameData(pParams->pGameData);
	return pObject;
}

CPhysicsObject::CPhysicsObject() {
	m_contents = 0;
	m_pShadow = NULL;
}

CPhysicsObject::~CPhysicsObject() {
	if (m_pEnv && m_pObject) {
		m_pEnv->GetBulletEnvironment()->removeRigidBody(m_pObject);
		delete m_pObject;
	}
	if (m_pShadow) {
		m_pObject->setMotionState(NULL);
		delete m_pShadow;
	}
}

bool CPhysicsObject::IsStatic() const {
	return GetMass() == 0.0;
}

bool CPhysicsObject::IsAsleep() const {
	return m_pObject->getActivationState() == ISLAND_SLEEPING;
}

bool CPhysicsObject::IsTrigger() const {
	NOT_IMPLEMENTED;
	return false;
}

bool CPhysicsObject::IsFluid() const {
	NOT_IMPLEMENTED;
	return false;
}

bool CPhysicsObject::IsHinged() const {
	NOT_IMPLEMENTED;
	return false;
}

bool CPhysicsObject::IsCollisionEnabled() const {
	NOT_IMPLEMENTED;
	return false;
}

bool CPhysicsObject::IsGravityEnabled() const {
	NOT_IMPLEMENTED;
	return false;
}

bool CPhysicsObject::IsDragEnabled() const {
	NOT_IMPLEMENTED;
	return false;
}

bool CPhysicsObject::IsMotionEnabled() const {
	return m_pObject->getActivationState() != DISABLE_SIMULATION;
}

bool CPhysicsObject::IsMoveable() const {
	if (IsStatic() || !IsMotionEnabled()) return false;
	return true;
}

bool CPhysicsObject::IsAttachedToConstraint(bool bExternalOnly) const {
	NOT_IMPLEMENTED;
	return false;
}

void CPhysicsObject::EnableCollisions(bool enable) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::EnableGravity(bool enable) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::EnableDrag(bool enable) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::EnableMotion(bool enable) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::SetGameData(void* pGameData) {
	m_pGameData = pGameData;
}

void* CPhysicsObject::GetGameData() const {
	return m_pGameData;
}

void CPhysicsObject::SetGameFlags(unsigned short userFlags) {
	m_gameFlags = userFlags;
}

unsigned short CPhysicsObject::GetGameFlags() const {
	return m_gameFlags;
}

void CPhysicsObject::SetGameIndex(unsigned short gameIndex) {
	NOT_IMPLEMENTED;
}

unsigned short CPhysicsObject::GetGameIndex() const {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsObject::SetCallbackFlags(unsigned short callbackflags) {
	m_callbacks = callbackflags;
}

unsigned short CPhysicsObject::GetCallbackFlags() const {
	return m_callbacks;
}

void CPhysicsObject::Wake() {
	m_pObject->setActivationState(ACTIVE_TAG);
}

void CPhysicsObject::Sleep() {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::RecheckCollisionFilter() {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::RecheckContactPoints() {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::SetMass(float mass) {
	NOT_IMPLEMENTED;
}

float CPhysicsObject::GetMass() const {
	btScalar invmass = m_pObject->getInvMass();
	if (invmass == 0) return 0;
	return 1.0/invmass;
}

float CPhysicsObject::GetInvMass() const {
	return m_pObject->getInvMass();
}

Vector CPhysicsObject::GetInertia() const {
	NOT_IMPLEMENTED;
	return Vector();
}

Vector CPhysicsObject::GetInvInertia() const {
	NOT_IMPLEMENTED;
	return Vector();
}

void CPhysicsObject::SetInertia(const Vector& inertia) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::SetDamping(const float* speed, const float* rot) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::GetDamping(float* speed, float* rot) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::SetDragCoefficient(float* pDrag, float* pAngularDrag) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::SetBuoyancyRatio(float ratio) {
	NOT_IMPLEMENTED;
}

int CPhysicsObject::GetMaterialIndex() const {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsObject::SetMaterialIndex(int materialIndex) {
	NOT_IMPLEMENTED;
}

unsigned int CPhysicsObject::GetContents() const {
	return m_contents;
}

void CPhysicsObject::SetContents(unsigned int contents) {
	m_contents = contents;
}

float CPhysicsObject::GetSphereRadius() const {
	NOT_IMPLEMENTED;
	return 0;
}

float CPhysicsObject::GetEnergy() const {
	NOT_IMPLEMENTED;
	return 0;
}

Vector CPhysicsObject::GetMassCenterLocalSpace() const {
	NOT_IMPLEMENTED;
	return Vector();
}

void CPhysicsObject::SetPosition(const Vector& worldPosition, const QAngle& angles, bool isTeleport) {
	btVector3 pos;
	btQuaternion rot;
	ConvertPosToBull(worldPosition, pos);
	ConvertRotationToBull(angles, rot);
	btTransform transform;
	transform.setOrigin(pos);
	transform.setRotation(rot);
	m_pObject->setWorldTransform(transform);
}

void CPhysicsObject::SetPositionMatrix(const matrix3x4_t&matrix, bool isTeleport) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::GetPosition(Vector* worldPosition, QAngle* angles) const {
	btTransform transform = m_pObject->getWorldTransform();
	if (worldPosition) ConvertPosToHL(transform.getOrigin(), *worldPosition);
	if (angles) ConvertRotationToHL(transform.getBasis(), *angles);
}

void CPhysicsObject::GetPositionMatrix(matrix3x4_t* positionMatrix) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::SetVelocity(const Vector* velocity, const AngularImpulse* angularVelocity) {
	btVector3 vel, angvel;
	if (velocity) {
		ConvertPosToBull(*velocity, vel);
		m_pObject->setLinearVelocity(vel);
	}
	if (angularVelocity) {
		ConvertPosToBull(*angularVelocity, angvel);
		m_pObject->setAngularVelocity(angvel);
	}
}

void CPhysicsObject::SetVelocityInstantaneous(const Vector* velocity, const AngularImpulse* angularVelocity) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::GetVelocity(Vector* velocity, AngularImpulse* angularVelocity) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::AddVelocity(const Vector* velocity, const AngularImpulse* angularVelocity) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::GetVelocityAtPoint(const Vector& worldPosition, Vector* pVelocity) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::GetImplicitVelocity(Vector* velocity, AngularImpulse* angularVelocity) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::LocalToWorld(Vector* worldPosition, const Vector& localPosition) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::WorldToLocal(Vector* localPosition, const Vector& worldPosition) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::LocalToWorldVector(Vector* worldVector, const Vector& localVector) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::WorldToLocalVector(Vector* localVector, const Vector& worldVector) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::ApplyForceCenter(const Vector& forceVector) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::ApplyForceOffset(const Vector& forceVector, const Vector& worldPosition) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::ApplyTorqueCenter(const AngularImpulse& torque) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::CalculateForceOffset(const Vector& forceVector, const Vector& worldPosition, Vector* centerForce, AngularImpulse* centerTorque) const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::CalculateVelocityOffset(const Vector& forceVector, const Vector& worldPosition, Vector* centerVelocity, AngularImpulse* centerAngularVelocity) const {
	NOT_IMPLEMENTED;
}

float CPhysicsObject::CalculateLinearDrag(const Vector& unitDirection) const {
	NOT_IMPLEMENTED;
	return 0;
}

float CPhysicsObject::CalculateAngularDrag(const Vector& objectSpaceRotationAxis) const {
	NOT_IMPLEMENTED;
	return 0;
}

bool CPhysicsObject::GetContactPoint(Vector* contactPoint, IPhysicsObject* *contactObject) const {
	NOT_IMPLEMENTED;
	return false;
}

void CPhysicsObject::SetShadow(float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation) {
	if (m_pShadow) {
		m_pShadow->MaxSpeed(maxSpeed, maxAngularSpeed);
	} else {
		unsigned int flags = GetCallbackFlags() | CALLBACK_SHADOW_COLLISION;
		flags &= ~CALLBACK_GLOBAL_FRICTION;
		flags &= ~CALLBACK_GLOBAL_COLLIDE_STATIC;
		SetCallbackFlags(flags);

		m_pShadow = (CShadowController*)m_pEnv->CreateShadowController(this, allowPhysicsMovement, allowPhysicsRotation);
		m_pShadow->MaxSpeed(maxSpeed, maxAngularSpeed);
	}
}

void CPhysicsObject::UpdateShadow(const Vector& targetPosition, const QAngle& targetAngles, bool tempDisableGravity, float timeOffset) {
	if (m_pShadow) {
		m_pShadow->Update(targetPosition, targetAngles, timeOffset);
	}
}

int CPhysicsObject::GetShadowPosition(Vector* position, QAngle* angles) const {
	NOT_IMPLEMENTED;
	return 0;
}

IPhysicsShadowController* CPhysicsObject::GetShadowController() const {
	return m_pShadow;
}

void CPhysicsObject::RemoveShadowController() {
	NOT_IMPLEMENTED;
}

float CPhysicsObject::ComputeShadowControl(const hlshadowcontrol_params_t& params, float secondsToArrival, float dt) {
	NOT_IMPLEMENTED;
	return 0;
}

const CPhysCollide* CPhysicsObject::GetCollide() const {
	return (CPhysCollide*)m_pObject->getCollisionShape();
}

const char* CPhysicsObject::GetName() const {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsObject::BecomeTrigger() {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::RemoveTrigger() {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::BecomeHinged(int localAxis) {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::RemoveHinged() {
	NOT_IMPLEMENTED;
}

IPhysicsFrictionSnapshot* CPhysicsObject::CreateFrictionSnapshot() {
	return new CPhysicsFrictionSnapshot;
}

void CPhysicsObject::DestroyFrictionSnapshot(IPhysicsFrictionSnapshot* pSnapshot) {
	delete pSnapshot;
}

void CPhysicsObject::OutputDebugInfo() const {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::Init(CPhysicsEnvironment* pEnv, btRigidBody* pObject) {
	m_pEnv = pEnv;
	m_pObject = pObject;
	pObject->setUserPointer(this);
	m_pGameData = NULL;
	m_gameFlags = 0;
	m_callbacks = CALLBACK_GLOBAL_COLLISION|CALLBACK_GLOBAL_FRICTION|CALLBACK_FLUID_TOUCH|CALLBACK_GLOBAL_TOUCH|CALLBACK_GLOBAL_COLLIDE_STATIC|CALLBACK_DO_FLUID_SIMULATION;
}

btRigidBody* CPhysicsObject::GetObject() {
	return m_pObject;
}