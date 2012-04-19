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
	info.m_angularDamping = pParams->rotdamping;
	info.m_localInertia = btVector3(pParams->inertia, pParams->inertia, pParams->inertia);

	btRigidBody* body = new btRigidBody(info);

	pEnvironment->GetBulletEnvironment()->addRigidBody(body);

	CPhysicsObject *pObject = new CPhysicsObject();
	pObject->Init(pEnvironment, body);
	pObject->SetGameData(pParams->pGameData);
	pObject->EnableCollisions(pParams->enableCollisions);
	
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
	return (m_pObject->getCollisionFlags() &  btCollisionObject::CF_STATIC_OBJECT);
}

bool CPhysicsObject::IsAsleep() const {
	return m_pObject->getActivationState() == ISLAND_SLEEPING;
}

bool CPhysicsObject::IsTrigger() const {
	NOT_IMPLEMENTED;
	return false;
}

bool CPhysicsObject::IsFluid() const {
	NOT_IMPLEMENTED;// based on material index??
	return false;
}

bool CPhysicsObject::IsHinged() const {
	NOT_IMPLEMENTED;
	return false;
}

bool CPhysicsObject::IsCollisionEnabled() const {
	return !(m_pObject->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE);
}

bool CPhysicsObject::IsGravityEnabled() const {
	return !(m_pObject->getFlags() & BT_DISABLE_WORLD_GRAVITY);
}

bool CPhysicsObject::IsDragEnabled() const {
	//return (bool)(m_pObject->getLinearDamping() + m_pObject->getAngularDamping());
	NOT_IMPLEMENTED;
}

bool CPhysicsObject::IsMotionEnabled() const {

	return (
		(m_pObject->getActivationState() != DISABLE_SIMULATION) || 
		(!(m_pObject->getCollisionFlags() & btRigidBody::CF_STATIC_OBJECT)) ||
		IsAsleep()
	); // it doesn't have to be simulation disabled to be motion disabled
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
	if (IsCollisionEnabled() != enable)
	{
		m_pObject->setCollisionFlags(m_pObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
	}
}

void CPhysicsObject::EnableGravity(bool enable) {
	if (IsGravityEnabled() != enable)
	{
		m_pObject->setFlags(m_pObject->getFlags() | BT_DISABLE_WORLD_GRAVITY);
	}
}

void CPhysicsObject::EnableDrag(bool enable) {
	NOT_IMPLEMENTED; // Damping
}

void CPhysicsObject::EnableMotion(bool enable) {
	if((m_pObject->getCollisionFlags() & btRigidBody::CF_STATIC_OBJECT) != enable)
	{
		m_pObject->setCollisionFlags(m_pObject->getCollisionFlags() | btRigidBody::CF_STATIC_OBJECT); // disabling simuation will make it not respond to other objects rather then being unable to move
	}
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
	m_pObject->setActivationState(ISLAND_SLEEPING);
}

void CPhysicsObject::RecheckCollisionFilter() {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::RecheckContactPoints() {
	NOT_IMPLEMENTED;
}

void CPhysicsObject::SetMass(float mass) {
	m_pObject->setMassProps(mass, m_pObject->getInvInertiaDiagLocal()); // not sure about this one
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
	btVector3 vec = m_pObject->getInvInertiaDiagLocal();
	Vector hl2vec;
	ConvertPosToHL(vec, hl2vec);
	return hl2vec;
}

void CPhysicsObject::SetInertia(const Vector& inertia) {
	btVector3 bull_inertia;
	ConvertPosToBull(inertia, bull_inertia);
	m_pObject->setInvInertiaDiagLocal(bull_inertia);
	m_pObject->updateInertiaTensor();
}

void CPhysicsObject::SetDamping(const float* speed, const float* rot) {
	m_pObject->setDamping(*speed, *rot);
}

void CPhysicsObject::GetDamping(float* speed, float* rot) const {
	*speed = m_pObject->getLinearDamping();
	*rot = m_pObject->getAngularDamping();
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
	btCollisionShape * shape = m_pObject->getCollisionShape();
	if (shape->getShapeType() != CYLINDER_SHAPE_PROXYTYPE)
		return 0;
	btCylinderShape * Cylinder_shape = (btCylinderShape *)shape;
	return Cylinder_shape->getRadius();
}

float CPhysicsObject::GetEnergy() const {
	float e = 0.5 * GetMass() * m_pObject->getLinearVelocity().dot(m_pObject->getLinearVelocity());
	e =+ 0.5 * GetMass() * m_pObject->getAngularVelocity().dot(m_pObject->getAngularVelocity());
	return ConvertEnergyToHL(e);
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
	ConvertPosToHL(m_pObject->getLinearVelocity(), *velocity);
	ConvertPosToHL(m_pObject->getAngularVelocity(), *angularVelocity);
}

void CPhysicsObject::AddVelocity(const Vector* velocity, const AngularImpulse* angularVelocity) {
	Vector CurrentVel, CurrentAngVel;
	GetVelocity(&CurrentVel, &CurrentAngVel);
	SetVelocity(&(CurrentVel + *velocity), &(CurrentAngVel + *angularVelocity));
}

void CPhysicsObject::GetVelocityAtPoint(const Vector& worldPosition, Vector* pVelocity) const {
	btVector3 vec;
	ConvertPosToBull(worldPosition, vec);
	ConvertPosToHL(m_pObject->getVelocityInLocalPoint(vec), *pVelocity);
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