#include "StdAfx.h"

#include "CPhysicsEnvironment.h"
#include "CPhysicsObject.h"
#include "CShadowController.h"
#include "CPlayerController.h"
#include "CPhysicsFluidController.h"
#include "CPhysicsDragController.h"
#include "CPhysicsMotionController.h"
#include "convert.h"

class IDeleteQueueItem {
public:
	virtual void Delete() = 0;
};

template <typename T>
class CDeleteProxy : public IDeleteQueueItem {
public:
	CDeleteProxy(T* pItem) : m_pItem(pItem) {}
	virtual void Delete() { delete m_pItem; }
private:
	T* m_pItem;
};

class CDeleteQueue {
public:
	void Add(IDeleteQueueItem *pItem) {
		m_list.AddToTail(pItem);
	}
	template <typename T>
	void QueueForDelete(T* pItem) {
		Add(new CDeleteProxy<T>(pItem));
	}
	void DeleteAll() {
		for (int i = m_list.Count()-1; i >= 0; --i) {
			m_list[i]->Delete();
			delete m_list[i];
		}
		m_list.RemoveAll();
	}
private:
	CUtlVector<IDeleteQueueItem*> m_list;
};

class CCollisionSolver : public btOverlapFilterCallback {
public:
	CCollisionSolver() {m_pSolver = NULL;}
	void SetHandler(IPhysicsCollisionSolver *pSolver) {m_pSolver = pSolver;}
	virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const;
private:
	IPhysicsCollisionSolver* m_pSolver;
};

bool CCollisionSolver::needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const {
	btRigidBody* body0 = (btRigidBody*)proxy0->m_clientObject;
	btRigidBody* body1 = (btRigidBody*)proxy1->m_clientObject;
	CPhysicsObject* pObject0 = (CPhysicsObject*)body0->getUserPointer();
	CPhysicsObject* pObject1 = (CPhysicsObject*)body1->getUserPointer();
	if (pObject0 && pObject1) {
		if ((pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) && (pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;
		if ((pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) && (pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;
		if (m_pSolver && !m_pSolver->ShouldCollide(pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData())) return false;
	}
	return true;
}

void CPhysicsEnvironment_TickCallBack(btDynamicsWorld *world, btScalar timeStep)
{
	CPhysicsEnvironment * phy = (CPhysicsEnvironment *)(world->getWorldUserInfo());
	phy->BulletTick(timeStep);
}

CPhysicsEnvironment::CPhysicsEnvironment() {
	m_deleteQuick = false;
	m_queueDeleteObject = false;
	m_inSimulation = false;

	m_pCollisionSolver = new CCollisionSolver;

	m_pBulletConfiguration = new btDefaultCollisionConfiguration();
	m_pBulletDispatcher = new btCollisionDispatcher(m_pBulletConfiguration);
	m_pBulletBroadphase = new btDbvtBroadphase();
	m_pBulletSolver = new btSequentialImpulseConstraintSolver();
	m_pBulletEnvironment = new btDiscreteDynamicsWorld(m_pBulletDispatcher, m_pBulletBroadphase, m_pBulletSolver, m_pBulletConfiguration);


	m_pBulletEnvironment->getPairCache()->setOverlapFilterCallback(m_pCollisionSolver);
	m_pBulletEnvironment->setInternalTickCallback(CPhysicsEnvironment_TickCallBack, (void *)(this));

	m_pDeleteQueue = new CDeleteQueue;

	m_pPhysicsDragController = new CPhysicsDragController;

	m_physics_performanceparams = new physics_performanceparams_t;
	m_physics_performanceparams->Defaults();

}

CPhysicsEnvironment::~CPhysicsEnvironment() {
	SetQuickDelete(true);

	for (int i = m_objects.Count()-1; i >= 0; --i) {
		CPhysicsObject *pObject = (CPhysicsObject*)(m_objects[i]);
		delete pObject;
	}

	m_objects.RemoveAll();
	CleanupDeleteList();

	delete m_pDeleteQueue;

	delete m_pBulletEnvironment;
	delete m_pBulletSolver;
	delete m_pBulletBroadphase;
	delete m_pBulletDispatcher;
	delete m_pBulletConfiguration;

	delete m_physics_performanceparams;
}

void CPhysicsEnvironment::SetDebugOverlay(CreateInterfaceFn debugOverlayFactory) {
}

IVPhysicsDebugOverlay* CPhysicsEnvironment::GetDebugOverlay() {
	return NULL;
}

void CPhysicsEnvironment::SetGravity(const Vector& gravityVector) {
	btVector3 temp;
	ConvertPosToBull(gravityVector, temp);
	m_pBulletEnvironment->setGravity(temp);
}
void CPhysicsEnvironment::GetGravity(Vector* pGravityVector) const {
	btVector3 temp = m_pBulletEnvironment->getGravity();
	ConvertPosToHL(temp, *pGravityVector);
}

void CPhysicsEnvironment::SetAirDensity(float density) {
	m_pPhysicsDragController->SetAirDensity(density);
}

float CPhysicsEnvironment::GetAirDensity() const {
	return m_pPhysicsDragController->GetAirDensity();
}

IPhysicsObject* CPhysicsEnvironment::CreatePolyObject(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, pCollisionModel, materialIndex, position, angles, pParams, false);
	m_objects.AddToTail(pObject);
	return pObject;
}

IPhysicsObject* CPhysicsEnvironment::CreatePolyObjectStatic(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, pCollisionModel, materialIndex, position, angles, pParams, true);
	m_objects.AddToTail(pObject);
	return pObject;
}

IPhysicsObject* CPhysicsEnvironment::CreateSphereObject(float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, (CPhysCollide*)new btSphereShape(HL2BULL(radius)), materialIndex, position, angles, pParams, false);
	m_objects.AddToTail(pObject);
	return pObject;
}

void CPhysicsEnvironment::DestroyObject(IPhysicsObject* pObject) {
	m_objects.FindAndRemove(pObject);
	delete pObject;
}

IPhysicsFluidController* CPhysicsEnvironment::CreateFluidController(IPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	CPhysicsFluidController *pFluid = ::CreateFluidController( static_cast<CPhysicsObject *>(pFluidObject), pParams );
	m_fluids.AddToTail( pFluid );
	return pFluid;
}

void CPhysicsEnvironment::DestroyFluidController(IPhysicsFluidController* tbr) {
	m_fluids.FindAndRemove((CPhysicsFluidController * )tbr);
}

IPhysicsSpring* CPhysicsEnvironment::CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsEnvironment::DestroySpring(IPhysicsSpring*) {
	NOT_IMPLEMENTED;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {	
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsEnvironment::DestroyConstraint(IPhysicsConstraint*) {
	NOT_IMPLEMENTED;
}

IPhysicsConstraintGroup* CPhysicsEnvironment::CreateConstraintGroup(const constraint_groupparams_t &groupParams) {
	NOT_IMPLEMENTED;
	return NULL;
}
void CPhysicsEnvironment::DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup) {
	NOT_IMPLEMENTED;
}

IPhysicsShadowController* CPhysicsEnvironment::CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	return new CShadowController((CPhysicsObject*)pObject, allowTranslation, allowRotation);
}

void CPhysicsEnvironment::DestroyShadowController(IPhysicsShadowController* pShadow) {
	delete pShadow;
}

IPhysicsPlayerController* CPhysicsEnvironment::CreatePlayerController(IPhysicsObject* pObject) {
	return new CPlayerController((CPhysicsObject*)pObject);
}
void CPhysicsEnvironment::DestroyPlayerController(IPhysicsPlayerController* controller) {
	delete controller;
}

IPhysicsMotionController* CPhysicsEnvironment::CreateMotionController(IMotionEvent *pHandler) {
	return ::CreateMotionController(this, pHandler);
}

void CPhysicsEnvironment::DestroyMotionController(IPhysicsMotionController *pController) {
	delete pController;
}

IPhysicsVehicleController* CPhysicsEnvironment::CreateVehicleController(IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsEnvironment::DestroyVehicleController(IPhysicsVehicleController*) {
	NOT_IMPLEMENTED;
}

void CPhysicsEnvironment::SetCollisionSolver(IPhysicsCollisionSolver *pSolver) {
	m_pCollisionSolver->SetHandler(pSolver);
}

void CPhysicsEnvironment::Simulate(float deltaTime) {
	if (!m_pBulletEnvironment) return;
	if ( deltaTime > 1.0 || deltaTime < 0.0 ) {
		deltaTime = 0;
	} else if ( deltaTime > 0.1 ) {
		deltaTime = 0.1f;
	}
	m_inSimulation = true;
	if (deltaTime > 0.0001) {
		m_pBulletEnvironment->stepSimulation(deltaTime, 1, m_timestep);
	}
	m_inSimulation = false;
	if (!m_queueDeleteObject) {
		CleanupDeleteList();
	}
}

bool CPhysicsEnvironment::IsInSimulation() const {
	return m_inSimulation;
}

float CPhysicsEnvironment::GetSimulationTimestep() const {
	return m_timestep;
}

void CPhysicsEnvironment::SetSimulationTimestep(float timestep) {
	m_timestep = timestep;
}

float CPhysicsEnvironment::GetSimulationTime() const {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsEnvironment::ResetSimulationClock() {
	NOT_IMPLEMENTED;
}

float CPhysicsEnvironment::GetNextFrameTime() const {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsEnvironment::SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents) {
	m_pCollisionEvent = pCollisionEvents;
}

void CPhysicsEnvironment::SetObjectEventHandler(IPhysicsObjectEvent *pObjectEvents) {
	m_pObjectEvent = pObjectEvents;
}

void CPhysicsEnvironment::SetConstraintEventHandler(IPhysicsConstraintEvent *pConstraintEvents) {
	m_pConstraintEvent = pConstraintEvents;
}

void CPhysicsEnvironment::SetQuickDelete(bool bQuick) {
	m_deleteQuick = bQuick;
}

int CPhysicsEnvironment::GetActiveObjectCount() const {
	return m_pBulletEnvironment->getNumCollisionObjects();
}
void CPhysicsEnvironment::GetActiveObjects(IPhysicsObject **pOutputObjectList ) const {
	const btCollisionObjectArray objects = m_pBulletEnvironment->getCollisionObjectArray();
	for (int i = 0; i < objects.size(); i++) {
		pOutputObjectList[i] = (IPhysicsObject*)objects[i]->getUserPointer();
	}
}

const IPhysicsObject** CPhysicsEnvironment::GetObjectList(int *pOutputObjectCount ) const {
	NOT_IMPLEMENTED;
	return NULL;
}

bool CPhysicsEnvironment::TransferObject(IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment) {
	NOT_IMPLEMENTED;
	return false;
}

void CPhysicsEnvironment::CleanupDeleteList(void) {
	for (int i = 0; i < m_deadObjects.Count(); i++) {
		CPhysicsObject *pObject = (CPhysicsObject*)m_deadObjects.Element(i);

		delete pObject;
	}
	m_deadObjects.Purge();
	m_pDeleteQueue->DeleteAll();
}
void CPhysicsEnvironment::EnableDeleteQueue(bool enable) {
	m_queueDeleteObject = enable;
}

bool CPhysicsEnvironment::Save(const physsaveparams_t &params) {
	NOT_IMPLEMENTED;
	return false;
}

void CPhysicsEnvironment::PreRestore(const physprerestoreparams_t &params) {
	NOT_IMPLEMENTED;
}

bool CPhysicsEnvironment::Restore(const physrestoreparams_t &params) {
	NOT_IMPLEMENTED;
	return false;
}
void CPhysicsEnvironment::PostRestore() {
	NOT_IMPLEMENTED;
}

bool CPhysicsEnvironment::IsCollisionModelUsed(CPhysCollide *pCollide) const {
	NOT_IMPLEMENTED;
	return false;
}
	
void CPhysicsEnvironment::TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	NOT_IMPLEMENTED;
}

void CPhysicsEnvironment::SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	NOT_IMPLEMENTED;
}

void CPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t *pOutput) const {
	memcpy(pOutput, m_physics_performanceparams, sizeof(physics_performanceparams_t));
}
void CPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t *pSettings) {
	memcpy((void *)m_physics_performanceparams, pSettings, sizeof(physics_performanceparams_t));
}

void CPhysicsEnvironment::ReadStats(physics_stats_t *pOutput) {
	NOT_IMPLEMENTED;
}
void CPhysicsEnvironment::ClearStats() {
	NOT_IMPLEMENTED;
}

unsigned int CPhysicsEnvironment::GetObjectSerializeSize(IPhysicsObject *pObject) const {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsEnvironment::SerializeObjectToBuffer(IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize) {
	NOT_IMPLEMENTED;
}

IPhysicsObject* CPhysicsEnvironment::UnserializeObjectFromBuffer(void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsEnvironment::EnableConstraintNotify(bool bEnable) {
	NOT_IMPLEMENTED; // UNDONE: Unsure what this functions is supposed to do its not documentated anywhere so for the moment it shall just be disabled
	// Andrew; this tells the physics environment to handle a callback whenever a physics object is removed that was attached to a constraint
	return;
}

void CPhysicsEnvironment::DebugCheckContacts(void) {
	NOT_IMPLEMENTED;
}

btDynamicsWorld* CPhysicsEnvironment::GetBulletEnvironment() {
	return m_pBulletEnvironment;
}
void CPhysicsEnvironment::BulletTick(btScalar dt)
{
	m_pPhysicsDragController->Tick(dt);
}
CPhysicsDragController * CPhysicsEnvironment::GetDragController()
{
	return m_pPhysicsDragController;
}