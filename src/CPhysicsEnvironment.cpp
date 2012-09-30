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

// WARNING: ATTEMPTING TO USE MULTITHREADING MAY CAUSE BRAINDAMGE DUE TO THE COMPLEXITY OF BUILDING BulletMultiThreaded.lib
// Looks like the person who wrote the above just had some trouble with CMake. Compiling the library was piss easy.
#define MULTITHREAD 1 // TODO: Mac and Linux support (Possibly done, needs testing)

#if MULTITHREAD
#	include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#	include "BulletMultiThreaded/PlatformDefinitions.h"
#	include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#	include "BulletMultiThreaded/btParallelConstraintSolver.h"

#	ifdef _WIN32
#		include "BulletMultiThreaded/Win32ThreadSupport.h"
#	else
#		include "BulletMultiThreaded/PosixThreadSupport.h"
#	endif

#	ifdef _DEBUG
#		pragma comment(lib, "BulletMultiThreaded_Debug.lib")
#	elif _RELEASE
#		pragma comment(lib, "BulletMultiThreaded_RelWithDebugInfo.lib")
#	endif
#endif

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

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
	btRigidBody* body0 = btRigidBody::upcast((btCollisionObject*)proxy0->m_clientObject);
	btRigidBody* body1 =  btRigidBody::upcast((btCollisionObject*)proxy1->m_clientObject);
	if (!body0 || !body1)
	{
		if (body0)
			return !(body0->isStaticObject() || body0->getInvMass() == 0.0f);
		if (body1)
			return !(body1->isStaticObject() || body1->getInvMass() == 0.0f);
		return false;
	}

	CPhysicsObject* pObject0 = (CPhysicsObject*)body0->getUserPointer();
	CPhysicsObject* pObject1 = (CPhysicsObject*)body1->getUserPointer();

	if (!pObject0 || !pObject1)
		return true;
	if (!pObject0->IsMoveable() && !pObject1->IsMoveable())
		return false;

	if ((pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) && (pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;
	if ((pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) && (pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;

	// FIXME: This is completely broken
	//if (m_pSolver && !m_pSolver->ShouldCollide(pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData())) return false;
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
	m_pDebugOverlay = NULL;

#if MULTITHREAD
	m_pThreadSupportCollision = NULL;
	m_pThreadSupportSolver = NULL;

	int maxTasks = 4;

	// HACK: Crash fix on the client (2 environments with same thread unique name = uh ohs)
	static int uniquenum = 0;	// Fixes a crash with multiple environments.
	char uniquenamecollision[1024];
	char uniquenamesolver[1024];
	sprintf(uniquenamecollision, "collision%d", uniquenum);
	sprintf(uniquenamesolver, "solver%d", uniquenum);
	uniquenum++;

#	ifdef _WIN32
	btThreadSupportInterface *threadInterface = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
																		uniquenamecollision,
																		processCollisionTask,
																		createCollisionLocalStoreMemory,
																		maxTasks));

	btThreadSupportInterface *solverThreadInterface = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
																		uniquenamesolver,
																		SolverThreadFunc,
																		SolverlsMemoryFunc,
																		maxTasks));
	solverThreadInterface->startSPU();
#	else
	btThreadSupportInterface *threadInterface = new PosixThreadSupport(PosixThreadSupport::PosixThreadConstructionInfo(
																		uniquenamecollision,
																		processCollisionTask,
																		createCollisionLocalStoreMemory,
																		maxTasks));

	btThreadSupportInterface *solverThreadInterface = new PosixThreadSupport(PosixThreadSupport::PosixThreadConstructionInfo(
																		uniquenamesolver,
																		SolverThreadFunc,
																		SolverlsMemoryFunc,
																		maxTasks));
#	endif

	m_pThreadSupportCollision = threadInterface;
	m_pThreadSupportSolver = solverThreadInterface;
	m_pBulletConfiguration = new btDefaultCollisionConfiguration();
	m_pBulletDispatcher = new SpuGatheringCollisionDispatcher(threadInterface, maxTasks, m_pBulletConfiguration);
	m_pBulletSolver = new btSequentialImpulseConstraintSolver();

	//m_pBulletSolver = new btParallelConstraintSolver(solverThreadInterface); // TODO: Work out bugs
#else
	m_pBulletConfiguration = new btDefaultCollisionConfiguration();
	m_pBulletDispatcher = new btCollisionDispatcher(m_pBulletConfiguration);
	m_pBulletSolver = new btSequentialImpulseConstraintSolver();
#endif

	m_pBulletBroadphase = new btDbvtBroadphase();
	m_pBulletEnvironment = new btDiscreteDynamicsWorld(m_pBulletDispatcher, m_pBulletBroadphase, m_pBulletSolver, m_pBulletConfiguration);

	m_pCollisionSolver = new CCollisionSolver;
	m_pBulletEnvironment->getPairCache()->setOverlapFilterCallback(m_pCollisionSolver);
	m_pBulletBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

	m_pDeleteQueue = new CDeleteQueue;

	m_pPhysicsDragController = new CPhysicsDragController;

	m_physics_performanceparams = new physics_performanceparams_t;
	m_physics_performanceparams->Defaults();

	m_pBulletEnvironment->getSolverInfo().m_numIterations = 4;
	m_pBulletEnvironment->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;
	m_pBulletEnvironment->getDispatchInfo().m_enableSPU = true;

	//m_simPSIs = 0;
	//m_invPSIscale = 0;

	m_pBulletEnvironment->setInternalTickCallback(CPhysicsEnvironment_TickCallBack, (void *)(this));

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
		CPhysicsObject *pObject = (CPhysicsObject*)(m_objects[i]);
		delete pObject;
	}

	m_objects.RemoveAll();
	CleanupDeleteList();

#if MULTITHREAD
	deleteCollisionLocalStoreMemory();
	delete m_pThreadSupportCollision;
	delete m_pThreadSupportSolver;
#endif

	delete m_pDeleteQueue;
	delete m_pPhysicsDragController;

	delete m_pBulletEnvironment;
	delete m_pBulletSolver;
	delete m_pBulletBroadphase;
	delete m_pBulletDispatcher;		// CRASH HERE
	delete m_pBulletConfiguration;

	delete m_physics_performanceparams;
}

void CPhysicsEnvironment::SetDebugOverlay(CreateInterfaceFn debugOverlayFactory) {
	m_pDebugOverlay = (IVPhysicsDebugOverlay *)debugOverlayFactory(VPHYSICS_DEBUG_OVERLAY_INTERFACE_VERSION, NULL);
}

IVPhysicsDebugOverlay *CPhysicsEnvironment::GetDebugOverlay() {
	return m_pDebugOverlay;
}

void CPhysicsEnvironment::SetGravity(const Vector& gravityVector) {
	btVector3 temp;
	ConvertPosToBull(gravityVector, temp);

	// There's unusually low gravity at default!
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
	IPhysicsObject *pObject = CreatePhysicsSphere(this, radius, materialIndex, position, angles, pParams, false);
	m_objects.AddToTail(pObject);
	return pObject;
}

void CPhysicsEnvironment::DestroyObject(IPhysicsObject* pObject) {
	if (!pObject) return;
	m_objects.FindAndRemove(pObject);
	if (m_inSimulation || m_queueDeleteObject) {
		pObject->SetCallbackFlags(pObject->GetCallbackFlags() | CALLBACK_MARKED_FOR_DELETE);
		m_deadObjects.AddToTail(pObject);
	} else {
		delete pObject;
	}
}

IPhysicsFluidController* CPhysicsEnvironment::CreateFluidController(IPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	CPhysicsFluidController *pFluid = ::CreateFluidController(this, static_cast<CPhysicsObject*>(pFluidObject), pParams);
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

IPhysicsConstraint* CPhysicsEnvironment::CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) 
{
	btTransform obj1Pos, obj2Pos;
	ConvertMatrixToBull(ragdoll.constraintToAttached, obj1Pos);
	ConvertMatrixToBull(ragdoll.constraintToReference, obj2Pos);
	CPhysicsObject *obj1 = (CPhysicsObject*)pReferenceObject, *obj2 = (CPhysicsObject*)pAttachedObject;

	btPoint2PointConstraint *ballsock = new btPoint2PointConstraint(*obj1->GetObject(), *obj2->GetObject(), obj1Pos.getOrigin(), obj2Pos.getOrigin());
	m_pBulletEnvironment->addConstraint(ballsock, false);
	return new CPhysicsConstraint(this, obj1, obj2, ballsock);
}

IPhysicsConstraint* CPhysicsEnvironment::CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed)
{
	CPhysicsObject *obj1 = (CPhysicsObject*)pReferenceObject, *obj2 = (CPhysicsObject*)pAttachedObject;
	btGeneric6DofConstraint *weld = new btGeneric6DofConstraint(*obj1->GetObject(), *obj2->GetObject(),
		obj1->GetObject()->getWorldTransform().inverse() * obj2->GetObject()->getWorldTransform(), btTransform::getIdentity(), true);
	weld->setLinearLowerLimit(btVector3(0,0,0));
	weld->setLinearUpperLimit(btVector3(0,0,0));
	weld->setAngularLowerLimit(btVector3(0,0,0));
	weld->setAngularUpperLimit(btVector3(0,0,0));
	m_pBulletEnvironment->addConstraint(weld, false);

	return new CPhysicsConstraint(this, obj1, obj2, weld);
}

IPhysicsConstraint* CPhysicsEnvironment::CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding)
{
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {	
	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(ballsocket.constraintPosition[0], obj1Pos);
	ConvertPosToBull(ballsocket.constraintPosition[1], obj2Pos);

	CPhysicsObject *obj1 = (CPhysicsObject*)pReferenceObject, *obj2 = (CPhysicsObject*)pAttachedObject;
	PhysicsShapeInfo *shapeInfo1 = (PhysicsShapeInfo*)obj1->GetObject()->getCollisionShape()->getUserPointer();
	PhysicsShapeInfo *shapeInfo2 = (PhysicsShapeInfo*)obj2->GetObject()->getCollisionShape()->getUserPointer();

	if (shapeInfo1)
		obj1Pos -= shapeInfo1->massCenter;
	if (shapeInfo2)
		obj2Pos -= shapeInfo2->massCenter;

	btPoint2PointConstraint *ballsock = new btPoint2PointConstraint(*obj1->GetObject(), *obj2->GetObject(), obj1Pos, obj2Pos);
	m_pBulletEnvironment->addConstraint(ballsock, false);
	return new CPhysicsConstraint(this, obj1, obj2, ballsock);
}

IPhysicsConstraint* CPhysicsEnvironment::CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	NOT_IMPLEMENTED;
	return NULL;
}

IPhysicsConstraint* CPhysicsEnvironment::CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(length.objectPosition[0], obj1Pos);
	ConvertPosToBull(length.objectPosition[1], obj2Pos);

	CPhysicsObject *obj1 = (CPhysicsObject*)pReferenceObject, *obj2 = (CPhysicsObject*)pAttachedObject;
	PhysicsShapeInfo *shapeInfo1 = (PhysicsShapeInfo*)obj1->GetObject()->getCollisionShape()->getUserPointer();
	PhysicsShapeInfo *shapeInfo2 = (PhysicsShapeInfo*)obj2->GetObject()->getCollisionShape()->getUserPointer();

	if (shapeInfo1)
		obj1Pos -= shapeInfo1->massCenter;
	if (shapeInfo2)
		obj2Pos -= shapeInfo2->massCenter;

	btPoint2PointConstraint *constraint = new btDistanceConstraint(*obj1->GetObject(), *obj2->GetObject(), obj1Pos, obj2Pos, HL2BULL(length.minLength), HL2BULL(length.totalLength));
	m_pBulletEnvironment->addConstraint(constraint, false);
	return new CPhysicsConstraint(this, obj1, obj2, constraint);
}

void CPhysicsEnvironment::DestroyConstraint(IPhysicsConstraint *pConstraint) {
	CPhysicsConstraint *constraint = (CPhysicsConstraint *)pConstraint;
	delete constraint;
}

IPhysicsConstraintGroup* CPhysicsEnvironment::CreateConstraintGroup(const constraint_groupparams_t &groupParams) {
	return new CPhysicsConstraintGroup();
}

void CPhysicsEnvironment::DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup) {
	delete (CPhysicsConstraintGroup *)pGroup;
}

IPhysicsShadowController* CPhysicsEnvironment::CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	CShadowController* pController = new CShadowController((CPhysicsObject*)pObject, allowTranslation, allowRotation);
	m_controllers.AddToTail(pController);
	return pController;
}

void CPhysicsEnvironment::DestroyShadowController(IPhysicsShadowController *pController) {
	m_controllers.FindAndRemove((CShadowController*)pController);
	delete pController;
}

IPhysicsPlayerController* CPhysicsEnvironment::CreatePlayerController(IPhysicsObject *pObject) {
	CPlayerController* pController = new CPlayerController((CPhysicsObject *)pObject);
	m_controllers.AddToTail(pController);
	return pController;
}

void CPhysicsEnvironment::DestroyPlayerController(IPhysicsPlayerController *pController) {
	m_controllers.FindAndRemove((CPlayerController*)pController);
	delete pController;
}

IPhysicsMotionController* CPhysicsEnvironment::CreateMotionController(IMotionEvent *pHandler) {
	CPhysicsMotionController* pController = (CPhysicsMotionController*)::CreateMotionController(this, pHandler);
	m_controllers.AddToTail(pController);
	return pController;
}

void CPhysicsEnvironment::DestroyMotionController(IPhysicsMotionController *pController) {
	m_controllers.FindAndRemove((CPhysicsMotionController*)pController);
	delete pController;
}

IPhysicsVehicleController* CPhysicsEnvironment::CreateVehicleController(IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	return ::CreateVehicleController(this, (CPhysicsObject *)pVehicleBodyObject, params, nVehicleType, pGameTrace);
}

void CPhysicsEnvironment::DestroyVehicleController(IPhysicsVehicleController *pController) {
	delete (CPhysicsVehicleController *)pController;
}

void CPhysicsEnvironment::SetCollisionSolver(IPhysicsCollisionSolver *pSolver) {
	m_pCollisionSolver->SetHandler(pSolver);
}

static ConVar cvar_numsubsteps("vphysics_maxsubsteps", "1", 0, "Sets the maximum amount of simulation substeps", true, 0, true, 1);
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
	if (deltaTime > 0.0001) {
		m_pBulletEnvironment->stepSimulation(deltaTime, cvar_numsubsteps.GetInt(), m_timestep/2.0f);
		for (int i = 0; i < m_fluids.Count(); i++) {
			m_fluids[i]->Tick(deltaTime);
		}

#if DEBUG_DRAW
		m_debugdraw->DrawWorld();
#endif
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
	return m_objects.Size();
}

void CPhysicsEnvironment::GetActiveObjects(IPhysicsObject **pOutputObjectList) const {
	int size = m_objects.Size();
	for (int i = 0; i < size; i++)
		pOutputObjectList[i] = m_objects[i];
}

// TODO: Not sure if this is correct and we're supposed to use m_objects.
const IPhysicsObject** CPhysicsEnvironment::GetObjectList(int *pOutputObjectCount) const {
	if (pOutputObjectCount) {
		*pOutputObjectCount = m_objects.Count();
	}

	return (const IPhysicsObject **)m_objects.Base();
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
	for (int i = 0; i < m_objects.Count(); i++) {
		if (((CPhysicsObject*)m_objects[i])->GetObject()->getCollisionShape() == (btCollisionShape*)pCollide)
			return true;
	}
	for (int i = 0; i < m_deadObjects.Count(); i++) {
		if (((CPhysicsObject*)m_deadObjects[i])->GetObject()->getCollisionShape() == (btCollisionShape*)pCollide)
			return true;
	}
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

float CPhysicsEnvironment::GetInvPSIScale() {
	//FIXME: get correct value
	//return m_invPSIscale;
	return 0.5;
}

void CPhysicsEnvironment::BulletTick(btScalar dt)
{
	// FIXME: Maybe this should be in CPhysicsEnvironment:Simulate instead?
	if (m_pCollisionEvent)
		m_pCollisionEvent->PostSimulationFrame();
	m_pPhysicsDragController->Tick(dt);
	for (int i = 0; i < m_controllers.Count(); i++) {
		m_controllers[i]->Tick(dt);
	}

	m_inSimulation = false;
	if (!m_queueDeleteObject) {
		CleanupDeleteList();
	}

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
	m_inSimulation = true;
}

CPhysicsDragController * CPhysicsEnvironment::GetDragController()
{
	return m_pPhysicsDragController;
}