#include "StdAfx.h"

#include "CPhysicsEnvironment.h"
#include "CPhysicsObject.h"
#include "CShadowController.h"
#include "CPlayerController.h"
#include "CPhysicsFluidController.h"
#include "CPhysicsDragController.h"
#include "CPhysicsMotionController.h"
#include "convert.h"
#include "CPhysicsConstraint.h"
#include "CPhysicsVehicleController.h"

#if DEBUG_DRAW
#	include "CDebugDrawer.h"
#endif

// Multithreading is so buggy as of now. Leave this disabled.
#define MULTITHREAD 0 // TODO: Mac and Linux support (Possibly done, needs testing)
#define USE_PARALLEL_SOLVER 0 // TODO: This crashes right now!
// See: http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=7016

#if MULTITHREAD
#	include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#	include "BulletMultiThreaded/PlatformDefinitions.h"
#	include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#	include "BulletMultiThreaded/btParallelConstraintSolver.h"
#
#	ifdef _WIN32
#		include "BulletMultiThreaded/Win32ThreadSupport.h"
#	else
#		include "BulletMultiThreaded/PosixThreadSupport.h"
#	endif
#
#	ifdef _DEBUG
#		pragma comment(lib, "BulletMultiThreaded_Debug.lib")
#	elif _RELEASE
#		pragma comment(lib, "BulletMultiThreaded_RelWithDebugInfo.lib")
#	endif
#endif

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/*****************************
* MISC. CLASSES
*****************************/

class IDeleteQueueItem {
	public:
		virtual void Delete() = 0;
};

template <typename T>
class CDeleteProxy : public IDeleteQueueItem {
	public:
		CDeleteProxy(T *pItem) : m_pItem(pItem) {}
		virtual void Delete() { delete m_pItem; }
	private:
		T *m_pItem;
};

class CDeleteQueue {
	public:
		void Add(IDeleteQueueItem *pItem) {
			m_list.AddToTail(pItem);
		}

		template <typename T>
		void QueueForDelete(T *pItem) {
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
		virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const;
	private:
		IPhysicsCollisionSolver *m_pSolver;
};

bool CCollisionSolver::needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const {
	btRigidBody *body0 = btRigidBody::upcast((btCollisionObject *)proxy0->m_clientObject);
	btRigidBody *body1 = btRigidBody::upcast((btCollisionObject *)proxy1->m_clientObject);
	if (!body0 || !body1)
	{
		if (body0)
			return !(body0->isStaticObject());
		if (body1)
			return !(body1->isStaticObject());
		return false;
	}

	CPhysicsObject *pObject0 = (CPhysicsObject *)body0->getUserPointer();
	CPhysicsObject *pObject1 = (CPhysicsObject *)body1->getUserPointer();

	if (!pObject0 || !pObject1)
		return true;

	if (!pObject0->IsCollisionEnabled() || !pObject1->IsCollisionEnabled())
		return false;

	// HACK: Fix shadow vs world collisions.
	if ((pObject0->GetShadowController() && pObject1->IsStatic()) || (pObject1->GetShadowController() && pObject0->IsStatic()))
		return false;

	if (pObject0->IsStatic() && pObject1->IsStatic())
		return false;

	if (pObject0->GetShadowController() && pObject1->GetShadowController())
		return false;

	if ((pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) && (pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;
	if ((pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) && (pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;

	// FIXME: This is completely broken
	//if (m_pSolver && !m_pSolver->ShouldCollide(pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData())) return false;
	return true;
}

/*******************************
* CLASS CPhysicsEnvironment
*******************************/

CPhysicsEnvironment::CPhysicsEnvironment() {
	m_deleteQuick = false;
	m_bUseDeleteQueue = false;
	m_inSimulation = false;
	m_pDebugOverlay = NULL;
	m_pConstraintEvent = NULL;

#if MULTITHREAD
	m_pThreadSupportCollision = NULL;
	m_pThreadSupportSolver = NULL;

	// TODO: In the future, try and get this value from the game somewhere.
	int maxTasks = 4;

	// HACK: Crash fix on the client (2 environments with same thread unique name = uh ohs)
	static unsigned int iUniqueNum = 0;	// Fixes a crash with multiple environments.
	char uniquenamecollision[128];
	char uniquenamesolver[128];
	sprintf(uniquenamecollision, "collision%d", iUniqueNum);
	sprintf(uniquenamesolver, "solver%d", iUniqueNum);
	iUniqueNum++;

#	ifdef _WIN32
	btThreadSupportInterface *threadInterface = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
																		uniquenamecollision,
																		processCollisionTask,
																		createCollisionLocalStoreMemory,
																		maxTasks));

#		if USE_PARALLEL_SOLVER
	btThreadSupportInterface *solverThreadInterface = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
																		uniquenamesolver,
																		SolverThreadFunc,
																		SolverlsMemoryFunc,
																		maxTasks));
	solverThreadInterface->startSPU();
#		endif
#	else
	btThreadSupportInterface *threadInterface = new PosixThreadSupport(PosixThreadSupport::PosixThreadConstructionInfo(
																		uniquenamecollision,
																		processCollisionTask,
																		createCollisionLocalStoreMemory,
																		maxTasks));

#		if USE_PARALLEL_SOLVER
	btThreadSupportInterface *solverThreadInterface = new PosixThreadSupport(PosixThreadSupport::PosixThreadConstructionInfo(
																		uniquenamesolver,
																		SolverThreadFunc,
																		SolverlsMemoryFunc,
																		maxTasks));
#		endif
#	endif

#	if USE_PARALLEL_SOLVER
	m_pThreadSupportSolver = solverThreadInterface;
#	endif

	m_pThreadSupportCollision = threadInterface;
	m_pBulletConfiguration = new btDefaultCollisionConfiguration();
	m_pBulletDispatcher = new SpuGatheringCollisionDispatcher(threadInterface, maxTasks, m_pBulletConfiguration);

#	if USE_PARALLEL_SOLVER
	m_pBulletSolver = new btParallelConstraintSolver(solverThreadInterface);
	//this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
	m_pBulletDispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);
#	else
	m_pBulletSolver = new btSequentialImpulseConstraintSolver();
#	endif // USE_PARARLLEL_SOLVER
#else
	m_pBulletConfiguration = new btDefaultCollisionConfiguration();
	m_pBulletDispatcher = new btCollisionDispatcher(m_pBulletConfiguration);
	m_pBulletSolver = new btSequentialImpulseConstraintSolver();
#endif // MULTITHREAD

	m_pBulletBroadphase = new btDbvtBroadphase();
	m_pBulletEnvironment = new btDiscreteDynamicsWorld(m_pBulletDispatcher, m_pBulletBroadphase, m_pBulletSolver, m_pBulletConfiguration);

	m_pCollisionSolver = new CCollisionSolver;
	m_pBulletEnvironment->getPairCache()->setOverlapFilterCallback(m_pCollisionSolver);
	m_pBulletBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());	// TODO: Does this leak memory?

	m_pDeleteQueue = new CDeleteQueue;

	m_pPhysicsDragController = new CPhysicsDragController;

	m_physics_performanceparams = new physics_performanceparams_t;
	m_physics_performanceparams->Defaults();

#if MULTITHREAD
	m_pBulletEnvironment->getSolverInfo().m_numIterations = 4;
	m_pBulletEnvironment->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;
	m_pBulletEnvironment->getDispatchInfo().m_enableSPU = true;
#endif

	//m_simPSIs = 0;
	//m_invPSIscale = 0;

	m_pBulletEnvironment->setInternalTickCallback(TickCallback, (void *)(this));

#if DEBUG_DRAW
	m_debugdraw = new CDebugDrawer(m_pBulletEnvironment, this);
#endif
}

CPhysicsEnvironment::~CPhysicsEnvironment() {
#if DEBUG_DRAW
	delete m_debugdraw;
#endif
	SetQuickDelete(true);

	for (int i = m_objects.Count()-1; i >= 0; --i) {
		CPhysicsObject *pObject = (CPhysicsObject *)(m_objects[i]);
		delete pObject;
	}

	m_objects.RemoveAll();
	CleanupDeleteList();

	delete m_pDeleteQueue;
	delete m_pPhysicsDragController;

	delete m_pBulletEnvironment;
	delete m_pBulletSolver;
	delete m_pBulletBroadphase;
	delete m_pBulletDispatcher;
	delete m_pBulletConfiguration;

#if MULTITHREAD
	// Remove the threads AFTER we remove what's using them!
	deleteCollisionLocalStoreMemory();
	delete m_pThreadSupportCollision;

#	if USE_PARALLEL_SOLVER
	delete m_pThreadSupportSolver;
#	endif
#endif

	delete m_physics_performanceparams;
}

// UNEXPOSED
void CPhysicsEnvironment::TickCallback(btDynamicsWorld *world, btScalar timeStep) {
	CPhysicsEnvironment *pEnv = (CPhysicsEnvironment *)(world->getWorldUserInfo());
	pEnv->BulletTick(timeStep);
}

void CPhysicsEnvironment::SetDebugOverlay(CreateInterfaceFn debugOverlayFactory) {
	m_pDebugOverlay = (IVPhysicsDebugOverlay *)debugOverlayFactory(VPHYSICS_DEBUG_OVERLAY_INTERFACE_VERSION, NULL);

	if (m_pDebugOverlay)
		m_debugdraw->SetDebugOverlay(m_pDebugOverlay);
}

IVPhysicsDebugOverlay *CPhysicsEnvironment::GetDebugOverlay() {
	return m_pDebugOverlay;
}

void CPhysicsEnvironment::SetGravity(const Vector& gravityVector) {
	btVector3 temp;
	ConvertPosToBull(gravityVector, temp);

	m_pBulletEnvironment->setGravity(temp);
}

void CPhysicsEnvironment::GetGravity(Vector *pGravityVector) const {
	btVector3 temp = m_pBulletEnvironment->getGravity();
	ConvertPosToHL(temp, *pGravityVector);
}

void CPhysicsEnvironment::SetAirDensity(float density) {
	m_pPhysicsDragController->SetAirDensity(density);
}

float CPhysicsEnvironment::GetAirDensity() const {
	return m_pPhysicsDragController->GetAirDensity();
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObject(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, pCollisionModel, materialIndex, position, angles, pParams, false);
	m_objects.AddToTail(pObject);
	return pObject;
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObjectStatic(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, pCollisionModel, materialIndex, position, angles, pParams, true);
	m_objects.AddToTail(pObject);
	return pObject;
}

IPhysicsObject *CPhysicsEnvironment::CreateSphereObject(float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic) {
	IPhysicsObject *pObject = CreatePhysicsSphere(this, radius, materialIndex, position, angles, pParams, isStatic);
	m_objects.AddToTail(pObject);
	return pObject;
}

void CPhysicsEnvironment::DestroyObject(IPhysicsObject *pObject) {
	if (!pObject) return;
	assert(m_deadObjects.Find(pObject) == -1);	// If you hit this assert, the object is already on the list!

	m_objects.FindAndRemove(pObject);

	if (m_inSimulation || m_bUseDeleteQueue) {
		((CPhysicsObject *)pObject)->AddCallbackFlags(CALLBACK_MARKED_FOR_DELETE);
		m_deadObjects.AddToTail(pObject);
	} else {
		delete pObject;
		pObject = NULL;
	}
}

IPhysicsFluidController *CPhysicsEnvironment::CreateFluidController(IPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	CPhysicsFluidController *pFluid = ::CreateFluidController(this, static_cast<CPhysicsObject*>(pFluidObject), pParams);
	m_fluids.AddToTail( pFluid );
	return pFluid;
}

void CPhysicsEnvironment::DestroyFluidController(IPhysicsFluidController *tbr) {
	m_fluids.FindAndRemove((CPhysicsFluidController  *)tbr);
}

IPhysicsSpring *CPhysicsEnvironment::CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsEnvironment::DestroySpring(IPhysicsSpring *pSpring) {
	NOT_IMPLEMENTED
}

IPhysicsConstraint *CPhysicsEnvironment::CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	return ::CreateRagdollConstraint(this, pReferenceObject, pAttachedObject, pGroup, ragdoll);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	return ::CreateHingeConstraint(this, pReferenceObject, pAttachedObject, pGroup, hinge);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	return ::CreateFixedConstraint(this, pReferenceObject, pAttachedObject, pGroup, fixed);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	return ::CreateSlidingConstraint(this, pReferenceObject, pAttachedObject, pGroup, sliding);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {
	return ::CreateBallsocketConstraint(this, pReferenceObject, pAttachedObject, pGroup, ballsocket);
}

IPhysicsConstraint *CPhysicsEnvironment::CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	return ::CreatePulleyConstraint(this, pReferenceObject, pAttachedObject, pGroup, pulley);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	return ::CreateLengthConstraint(this, pReferenceObject, pAttachedObject, pGroup, length);
}

void CPhysicsEnvironment::DestroyConstraint(IPhysicsConstraint *pConstraint) {
	if (!pConstraint) return;

	if (m_deleteQuick) {
		IPhysicsObject *pObj0 = pConstraint->GetReferenceObject();
		if (pObj0)
			pObj0->Wake();

		IPhysicsObject *pObj1 = pConstraint->GetAttachedObject();
		if (pObj1)
			pObj1->Wake();
	}

	if (m_inSimulation) {
		pConstraint->Deactivate();
		m_pDeleteQueue->QueueForDelete(pConstraint);
	} else {
		delete pConstraint;
	}
}

IPhysicsConstraintGroup *CPhysicsEnvironment::CreateConstraintGroup(const constraint_groupparams_t &groupParams) {
	return new CPhysicsConstraintGroup();
}

void CPhysicsEnvironment::DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup) {
	delete (CPhysicsConstraintGroup *)pGroup;
}

IPhysicsShadowController *CPhysicsEnvironment::CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	CShadowController *pController = new CShadowController((CPhysicsObject *)pObject, allowTranslation, allowRotation);
	m_controllers.AddToTail(pController);
	return pController;
}

void CPhysicsEnvironment::DestroyShadowController(IPhysicsShadowController *pController) {
	m_controllers.FindAndRemove((CShadowController *)pController);
	delete pController;
}

IPhysicsPlayerController *CPhysicsEnvironment::CreatePlayerController(IPhysicsObject *pObject) {
	CPlayerController *pController = new CPlayerController((CPhysicsObject *)pObject);
	m_controllers.AddToTail(pController);
	return pController;
}

void CPhysicsEnvironment::DestroyPlayerController(IPhysicsPlayerController *pController) {
	m_controllers.FindAndRemove((CPlayerController *)pController);
	delete pController;
}

IPhysicsMotionController *CPhysicsEnvironment::CreateMotionController(IMotionEvent *pHandler) {
	CPhysicsMotionController *pController = (CPhysicsMotionController *)::CreateMotionController(this, pHandler);
	m_controllers.AddToTail(pController);
	return pController;
}

void CPhysicsEnvironment::DestroyMotionController(IPhysicsMotionController *pController) {
	m_controllers.FindAndRemove((CPhysicsMotionController *)pController);
	delete pController;
}

IPhysicsVehicleController *CPhysicsEnvironment::CreateVehicleController(IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	return ::CreateVehicleController(this, (CPhysicsObject *)pVehicleBodyObject, params, nVehicleType, pGameTrace);
}

void CPhysicsEnvironment::DestroyVehicleController(IPhysicsVehicleController *pController) {
	delete (CPhysicsVehicleController *)pController;
}

void CPhysicsEnvironment::SetCollisionSolver(IPhysicsCollisionSolver *pSolver) {
	m_pCollisionSolver->SetHandler(pSolver);
}

static ConVar cvar_maxsubsteps("vphysics_maxsubsteps", "2", 0, "Sets the maximum amount of simulation substeps", true, 1, true, 8);
void CPhysicsEnvironment::Simulate(float deltaTime) {
	if (!m_pBulletEnvironment) return;

	if ( deltaTime > 1.0 || deltaTime < 0.0 ) {
		deltaTime = 0;
	} else if ( deltaTime > 0.1 ) {
		deltaTime = 0.1f;
	}

	//m_simPSIs = 0;

	//FIXME: figure out simulation time

	//m_simPSIcurrent = m_simPSIs;

	m_inSimulation = true;
	if (deltaTime > 1e-4) {
		// Divide by zero check.
		float timestep = cvar_maxsubsteps.GetInt() != 0 ? m_timestep / cvar_maxsubsteps.GetInt() : m_timestep;
		m_pBulletEnvironment->stepSimulation(deltaTime, cvar_maxsubsteps.GetInt(), timestep); // m_timestep/2.0f
	}
	m_inSimulation = false;

#if DEBUG_DRAW
	m_debugdraw->DrawWorld();
#endif

	if (!m_bUseDeleteQueue) {
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
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsEnvironment::ResetSimulationClock() {
	NOT_IMPLEMENTED
}

float CPhysicsEnvironment::GetNextFrameTime() const {
	NOT_IMPLEMENTED
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
	return m_objects.Size();
}

void CPhysicsEnvironment::GetActiveObjects(IPhysicsObject **pOutputObjectList) const {
	int size = m_objects.Size();
	for (int i = 0; i < size; i++)
		pOutputObjectList[i] = m_objects[i];
}

// TODO: Not sure if this is correct and we're supposed to use m_objects.
const IPhysicsObject **CPhysicsEnvironment::GetObjectList(int *pOutputObjectCount) const {
	if (pOutputObjectCount) {
		*pOutputObjectCount = m_objects.Count();
	}

	return (const IPhysicsObject **)m_objects.Base();
}

bool CPhysicsEnvironment::TransferObject(IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment) {
	if (pDestinationEnvironment == this) {
		m_pBulletEnvironment->addRigidBody(((CPhysicsObject *)pObject)->GetObject());
		m_objects.AddToTail(pObject);
		return true;
	} else {
		m_pBulletEnvironment->removeRigidBody(((CPhysicsObject *)pObject)->GetObject());
		m_objects.FindAndRemove(pObject);
		return pDestinationEnvironment->TransferObject(pObject, pDestinationEnvironment);
	}
}

void CPhysicsEnvironment::CleanupDeleteList(void) {
	for (int i = 0; i < m_deadObjects.Count(); i++) {
		CPhysicsObject *pObject = (CPhysicsObject *)m_deadObjects.Element(i);
		delete pObject;
	}

	m_deadObjects.Purge();
	m_pDeleteQueue->DeleteAll();
}

void CPhysicsEnvironment::EnableDeleteQueue(bool enable) {
	m_bUseDeleteQueue = enable;
}

bool CPhysicsEnvironment::Save(const physsaveparams_t &params) {
	NOT_IMPLEMENTED
	return false;
}

void CPhysicsEnvironment::PreRestore(const physprerestoreparams_t &params) {
	NOT_IMPLEMENTED
}

bool CPhysicsEnvironment::Restore(const physrestoreparams_t &params) {
	NOT_IMPLEMENTED
	return false;
}

void CPhysicsEnvironment::PostRestore() {
	NOT_IMPLEMENTED
}

bool CPhysicsEnvironment::IsCollisionModelUsed(CPhysCollide *pCollide) const {
	for (int i = 0; i < m_objects.Count(); i++) {
		if (((CPhysicsObject *)m_objects[i])->GetObject()->getCollisionShape() == (btCollisionShape *)pCollide)
			return true;
	}

	return false;
}
	
void CPhysicsEnvironment::TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	NOT_IMPLEMENTED
}

void CPhysicsEnvironment::SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	NOT_IMPLEMENTED
}

void CPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t *pOutput) const {
	memcpy(pOutput, m_physics_performanceparams, sizeof(physics_performanceparams_t));
}

void CPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t *pSettings) {
	memcpy((void *)m_physics_performanceparams, pSettings, sizeof(physics_performanceparams_t));
}

void CPhysicsEnvironment::ReadStats(physics_stats_t *pOutput) {
	NOT_IMPLEMENTED
}

void CPhysicsEnvironment::ClearStats() {
	NOT_IMPLEMENTED
}

unsigned int CPhysicsEnvironment::GetObjectSerializeSize(IPhysicsObject *pObject) const {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsEnvironment::SerializeObjectToBuffer(IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize) {
	NOT_IMPLEMENTED
}

IPhysicsObject *CPhysicsEnvironment::UnserializeObjectFromBuffer(void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsEnvironment::EnableConstraintNotify(bool bEnable) {
	NOT_IMPLEMENTED // UNDONE: Unsure what this functions is supposed to do its not documentated anywhere so for the moment it shall just be disabled
	// Andrew; this tells the physics environment to handle a callback whenever a physics object is removed that was attached to a constraint
	return;
}

void CPhysicsEnvironment::DebugCheckContacts(void) {
	NOT_IMPLEMENTED
}

btDynamicsWorld *CPhysicsEnvironment::GetBulletEnvironment() {
	return m_pBulletEnvironment;
}

float CPhysicsEnvironment::GetInvPSIScale() {
	//FIXME: get correct value
	//return m_invPSIscale;
	return 0.5;
}

void CPhysicsEnvironment::BulletTick(btScalar dt) {
	m_pPhysicsDragController->Tick(dt);
	for (int i = 0; i < m_controllers.Count(); i++)
		m_controllers[i]->Tick(dt);

	for (int i = 0; i < m_fluids.Count(); i++)
		m_fluids[i]->Tick(dt);

	m_inSimulation = false;
	if (!m_bUseDeleteQueue) {
		CleanupDeleteList();
	}

	DoCollisionEvents(dt);

	//m_pCollisionSolver->EventPSI(this);
	//m_pCollisionListener->EventPSI(this);

	/*
	if (m_simPSIcurrent) {
		m_invPSIscale = 1.0f / (float)m_simPSIcurrent;
		m_simPSIcurrent--;
	} else {
		m_invPSIscale = 0;
	}
	*/
	if (m_pCollisionEvent)
		m_pCollisionEvent->PostSimulationFrame();

	m_inSimulation = true;
}

CPhysicsDragController *CPhysicsEnvironment::GetDragController() {
	return m_pPhysicsDragController;
}

// UNEXPOSED
// TODO: Bullet exposes collision callbacks such as gContactAddedCallback!
// See: http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Callbacks_and_Triggers
void CPhysicsEnvironment::DoCollisionEvents(float dt) {
	// IPhysicsCollisionEvent::Friction
	/*
	int numManifolds = m_pBulletEnvironment->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold *contactManifold = m_pBulletEnvironment->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject *obA = contactManifold->getBody0();
		const btCollisionObject *obB = contactManifold->getBody1();

		if (contactManifold->getNumContacts() <= 0)
			continue;

		if (obA->getInternalType() == btCollisionObject::CO_GHOST_OBJECT || obB->getInternalType() == btCollisionObject::CO_GHOST_OBJECT)
			continue;

		if ((((CPhysicsObject *)obA->getUserPointer())->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION) &&
			(((CPhysicsObject *)obB->getUserPointer())->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION)) {
			btManifoldPoint manPoint = contactManifold->getContactPoint(0);
			
			float energy = manPoint.m_combinedFriction;
			if (energy > 0.05f) {
				
			}
		}
	}
	*/

	/*
	if (m_pObjectEvent)
	{
		// FIXME: This got very messy, must be a better way to do this
		int numObjects = m_pBulletEnvironment->getNumCollisionObjects();
		btCollisionObjectArray collisionObjects = m_pBulletEnvironment->getCollisionObjectArray();
		for (int i = 0; i < numObjects; i++)
		{
			btCollisionObject *obj = collisionObjects[i];
			CPhysicsObject *physobj = (CPhysicsObject*)collisionObjects[i]->getUserPointer();
			if (physobj->m_iLastActivationState != obj->getActivationState())
			{
				switch (obj->getActivationState())
				{
				case ACTIVE_TAG:
					m_pObjectEvent->ObjectWake(physobj);
					break;
				case ISLAND_SLEEPING:
					m_pObjectEvent->ObjectSleep(physobj);
					break;
				}
				physobj->m_iLastActivationState = obj->getActivationState();
			}
		}
	}
	*/
}