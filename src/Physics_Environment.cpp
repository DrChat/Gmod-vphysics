#include "StdAfx.h"

#include <cmodel.h>

#include "Physics_Environment.h"
#include "Physics.h"
#include "Physics_Object.h"
#include "Physics_ShadowController.h"
#include "Physics_PlayerController.h"
#include "Physics_FluidController.h"
#include "Physics_DragController.h"
#include "Physics_MotionController.h"
#include "Physics_Constraint.h"
#include "Physics_VehicleController.h"
#include "Physics_SoftBody.h"
#include "convert.h"

#include "tier0/vprof.h"

#if DEBUG_DRAW
	#include "DebugDrawer.h"
#endif

// Multithreading stuff
// NOTE: As of now, the parallel constraint solver has the same performance as the
// sequential constraint solver, and even in some cases is slower. The parallel dispatcher
// allows for some speed gain, however.
// We should try to create a parallel dynamics world that'll support multithreading, and
// use the sequential dispatcher and constraint solver, as that is where most of the effort
// in bullet went.

#define USE_PARALLEL_DISPATCHER 0
#define USE_PARALLEL_SOLVER 0

#if USE_PARALLEL_DISPATCHER || USE_PARALLEL_SOLVER
	#define MULTITHREADED 1
#endif

#if MULTITHREADED
	#ifdef _WIN32
		#include "BulletMultiThreaded/Win32ThreadSupport.h"
		#pragma comment(lib, "BulletMultiThreaded.lib")
	#else
		#include "BulletMultiThreaded/PosixThreadSupport.h"
	#endif

	#include "BulletMultiThreaded/PlatformDefinitions.h"
#endif

#if USE_PARALLEL_DISPATCHER
	#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
	#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif

#if USE_PARALLEL_SOLVER
	#include "BulletMultiThreaded/btParallelConstraintSolver.h"
#endif

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

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
		CUtlVector<IDeleteQueueItem *> m_list;
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
	if (!body0 || !body1) {
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

	if ((pObject0->GetObject()->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT && pObject1->IsStatic())
	 || (pObject0->GetObject()->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT && pObject0->IsStatic()))
		return false;

	if (pObject0->IsStatic() && pObject1->IsStatic())
		return false;

	if (pObject0->GetShadowController() && pObject1->GetShadowController())
		return false;

	if ((pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) && (pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;
	if ((pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) && (pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;

	if (m_pSolver && !m_pSolver->ShouldCollide(pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData())) return false;

	// And then the default bullet stuff...
	bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
	collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
	
	return collides;
}

/*******************************
* CLASS CPhysicsCollisionData
*******************************/

class CPhysicsCollisionData : public IPhysicsCollisionData {
	public:
		void GetSurfaceNormal(Vector &out);		// normal points toward second object (object index 1)
		void GetContactPoint(Vector &out);		// contact point of collision (in world space)
		void GetContactSpeed(Vector &out);		// speed of surface 1 relative to surface 0 (in world space)

		// UNEXPOSED FUNCTIONS
		CPhysicsCollisionData(btManifoldPoint *manPoint);

	private:
		Vector m_surfaceNormal;
		Vector m_contactPoint;
		Vector m_contactSpeed;
};

CPhysicsCollisionData::CPhysicsCollisionData(btManifoldPoint *manPoint) {
	ConvertDirectionToHL(manPoint->m_normalWorldOnB, m_surfaceNormal);
	ConvertPosToHL(manPoint->getPositionWorldOnA(), m_contactPoint);
	ConvertPosToHL(manPoint->m_lateralFrictionDir1, m_contactSpeed);	// FIXME: Need the correct variable from the manifold point
}

void CPhysicsCollisionData::GetSurfaceNormal(Vector &out) {
	out = m_surfaceNormal;
}

void CPhysicsCollisionData::GetContactPoint(Vector &out) {
	out = m_contactPoint;
}

void CPhysicsCollisionData::GetContactSpeed(Vector &out) {
	out = m_contactSpeed;
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
	m_pObjectEvent = NULL;
	m_pCollisionEvent = NULL;

	m_pThreadSupportDispatcher = NULL;
	m_pThreadSupportSolver = NULL;
	m_pBulletBroadphase = NULL;
	m_pBulletConfiguration = NULL;
	m_pBulletDispatcher = NULL;
	m_pBulletEnvironment = NULL;
	m_pBulletGhostCallback = NULL;
	m_pBulletSolver = NULL;

	btDefaultCollisionConstructionInfo cci;
#if MULTITHREADED
	// Multithreaded versions cannot dynamically expand this pool. Having a small pool causes props to bounce around.
	cci.m_defaultMaxPersistentManifoldPoolSize = 16384;

	// HACK: Fix to prevent crashing on games with more than 1 environment.
	static unsigned int uniqueNum = 0;

	// Maximum number of parallel tasks (number of threads in the thread support)
	// Good to set it to the same amount of CPU cores on the system.
	static const int maxTasks = 4;
#endif

	m_pBulletConfiguration = new btSoftBodyRigidBodyCollisionConfiguration(cci);

#if USE_PARALLEL_DISPATCHER
	char dispatcherThreadName[128];
	sprintf(dispatcherThreadName, "dispatcher%d", uniqueNum);

	#ifdef _WIN32
		m_pThreadSupportDispatcher = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
																		dispatcherThreadName,
																		processCollisionTask,
																		createCollisionLocalStoreMemory,
																		maxTasks));
	#else
		m_pThreadSupportDispatcher = new PosixThreadSupport(PosixThreadSupport::PosixThreadConstructionInfo(
																		dispatcherThreadName,
																		processCollisionTask,
																		createCollisionLocalStoreMemory,
																		maxTasks));
	#endif

	m_pBulletDispatcher = new SpuGatheringCollisionDispatcher(m_pThreadSupportDispatcher, maxTasks, m_pBulletConfiguration);
#else
	m_pBulletDispatcher = new btCollisionDispatcher(m_pBulletConfiguration);
#endif

#if USE_PARALLEL_SOLVER
	char solverThreadName[128];
	sprintf(solverThreadName, "solver%d", uniqueNum);

	#ifdef _WIN32
		m_pThreadSupportSolver = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
																			solverThreadName,
																			SolverThreadFunc,
																			SolverlsMemoryFunc,
																			maxTasks));
	#else
		m_pThreadSupportSolver = new PosixThreadSupport(PosixThreadSupport::PosixThreadConstructionInfo(
									 										solverThreadName,
									 										SolverThreadFunc,
									 										SolverlsMemoryFunc,
									 										maxTasks));
	#endif

	m_pBulletSolver = new btParallelConstraintSolver(m_pThreadSupportSolver);
#else
	m_pBulletSolver = new btSequentialImpulseConstraintSolver;
#endif

#if MULTITHREADED
	uniqueNum++;
#endif

	m_pBulletBroadphase = new btDbvtBroadphase;

	// Note: The soft body solver (last default-arg in the constructor) is used for OpenCL stuff (as per the Soft Body Demo)
	m_pBulletEnvironment = new btSoftRigidDynamicsWorld(m_pBulletDispatcher, m_pBulletBroadphase, m_pBulletSolver, m_pBulletConfiguration);

	m_pBulletGhostCallback = new btGhostPairCallback;
	m_pCollisionSolver = new CCollisionSolver;
	m_pBulletEnvironment->getPairCache()->setOverlapFilterCallback(m_pCollisionSolver);
	m_pBulletBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(m_pBulletGhostCallback);

	m_pDeleteQueue = new CDeleteQueue;

	m_pPhysicsDragController = new CPhysicsDragController;

	m_perfparams.Defaults();
	memset(&m_stats, 0, sizeof(m_stats));

#if MULTITHREADED
	m_pBulletEnvironment->getSolverInfo().m_numIterations = maxTasks;
	m_pBulletEnvironment->getDispatchInfo().m_enableSPU = true;
	#if USE_PARALLEL_SOLVER
		// Required for parallel solver (undocumented lolololo)
		m_pBulletEnvironment->getSimulationIslandManager()->setSplitIslands(false);

		// Solver also requires this.
		m_pBulletDispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);
	#endif
#endif

	m_pBulletEnvironment->getSolverInfo().m_solverMode |= SOLVER_SIMD | SOLVER_RANDMIZE_ORDER | SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_USE_WARMSTARTING;
	m_pBulletEnvironment->getDispatchInfo().m_useContinuous = true;
	m_pBulletEnvironment->getDispatchInfo().m_allowedCcdPenetration = 0.0001f;

	//m_simPSIs = 0;
	//m_invPSIscale = 0;

	m_pBulletEnvironment->setInternalTickCallback(TickCallback, (void *)(this));

#if DEBUG_DRAW
	m_debugdraw = new CDebugDrawer(m_pBulletEnvironment, this);
#endif

	// HACK: Durr hack to get ourselves a debug overlay on the client
	CreateInterfaceFn engine = Sys_GetFactory("engine");
	SetDebugOverlay(engine);
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
	delete m_pBulletGhostCallback;

	delete m_pCollisionSolver;

#if MULTITHREAD
	deleteCollisionLocalStoreMemory();
	delete m_pThreadSupportCollision;

	#if USE_PARALLEL_SOLVER
		delete m_pThreadSupportSolver;
	#endif
#endif
}

// UNEXPOSED
void CPhysicsEnvironment::TickCallback(btDynamicsWorld *world, btScalar timeStep) {
	if (!world) return;

	CPhysicsEnvironment *pEnv = (CPhysicsEnvironment *)world->getWorldUserInfo();
	if (pEnv)
		pEnv->BulletTick(timeStep);
}

void CPhysicsEnvironment::SetDebugOverlay(CreateInterfaceFn debugOverlayFactory) {
	if (!debugOverlayFactory) return;
	m_pDebugOverlay = (IVPhysicsDebugOverlay *)debugOverlayFactory(VPHYSICS_DEBUG_OVERLAY_INTERFACE_VERSION, NULL);

#if DEBUG_DRAW
	if (m_pDebugOverlay)
		m_debugdraw->SetDebugOverlay(m_pDebugOverlay);
#endif
}

IVPhysicsDebugOverlay *CPhysicsEnvironment::GetDebugOverlay() {
	return m_pDebugOverlay;
}

btIDebugDraw *CPhysicsEnvironment::GetDebugDrawer() {
	return (btIDebugDraw *)m_debugdraw;
}

void CPhysicsEnvironment::SetGravity(const Vector &gravityVector) {
	btVector3 temp;
	ConvertPosToBull(gravityVector, temp);

	m_pBulletEnvironment->setGravity(temp);
}

void CPhysicsEnvironment::GetGravity(Vector *pGravityVector) const {
	if (!pGravityVector) return;

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
	Assert(m_deadObjects.Find(pObject) == -1);	// If you hit this assert, the object is already on the list!

	m_objects.FindAndRemove(pObject);

	if (m_inSimulation || m_bUseDeleteQueue) {
		((CPhysicsObject *)pObject)->AddCallbackFlags(CALLBACK_MARKED_FOR_DELETE);
		m_deadObjects.AddToTail(pObject);
	} else {
		delete pObject;
		pObject = NULL;
	}
}

IPhysicsSoftBody *CPhysicsEnvironment::CreateSoftBodyFromVertices(const Vector *vertices, int numVertices, const Vector &position, const QAngle &angles) {
	CPhysicsSoftBody *pSoftBody = ::CreateSoftBodyFromVertices(this, vertices, numVertices, position, angles);
	m_softBodies.AddToTail(pSoftBody);
	return pSoftBody;
}

void CPhysicsEnvironment::DestroySoftBody(IPhysicsSoftBody *pSoftBody) {
	if (!pSoftBody) return;

	m_softBodies.FindAndRemove(pSoftBody);
	NOT_IMPLEMENTED
	
	if (m_inSimulation || m_bUseDeleteQueue) {
		// TODO
	} else {
		delete pSoftBody;
		pSoftBody = NULL;
	}
}

IPhysicsFluidController *CPhysicsEnvironment::CreateFluidController(IPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	CPhysicsFluidController *pFluid = ::CreateFluidController(this, (CPhysicsObject *)pFluidObject, pParams);
	m_fluids.AddToTail(pFluid);
	return pFluid;
}

void CPhysicsEnvironment::DestroyFluidController(IPhysicsFluidController *pController) {
	m_fluids.FindAndRemove((CPhysicsFluidController  *)pController);
	delete (CPhysicsFluidController *)pController;
}

IPhysicsSpring *CPhysicsEnvironment::CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams) {
	return ::CreateSpringConstraint(this, pObjectStart, pObjectEnd, pParams);
}

void CPhysicsEnvironment::DestroySpring(IPhysicsSpring *pSpring) {
	if (!pSpring) return;

	// REMEMBER: If you allocate anything inside IPhysicsSpring, you'll have to REWRITE THIS FUNCTION!!!
	DestroyConstraint((IPhysicsConstraint *)pSpring);
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
		if (pObj0 && !pObj0->IsStatic())
			pObj0->Wake();

		IPhysicsObject *pObj1 = pConstraint->GetAttachedObject();
		if (pObj1 && !pObj0->IsStatic())
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
	return new CPhysicsConstraintGroup(groupParams);
}

void CPhysicsEnvironment::DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup) {
	delete (CPhysicsConstraintGroup *)pGroup;
}

IPhysicsShadowController *CPhysicsEnvironment::CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	CShadowController *pController = ::CreateShadowController(pObject, allowTranslation, allowRotation);
	if (pController)
		m_controllers.AddToTail(pController);

	return pController;
}

void CPhysicsEnvironment::DestroyShadowController(IPhysicsShadowController *pController) {
	if (!pController) return;

	m_controllers.FindAndRemove((CShadowController *)pController);
	delete (CShadowController *)pController;
}

IPhysicsPlayerController *CPhysicsEnvironment::CreatePlayerController(IPhysicsObject *pObject) {
	CPlayerController *pController = ::CreatePlayerController(pObject);
	if (pController)
		m_controllers.AddToTail(pController);

	return pController;
}

void CPhysicsEnvironment::DestroyPlayerController(IPhysicsPlayerController *pController) {
	if (!pController) return;

	m_controllers.FindAndRemove((CPlayerController *)pController);
	delete (CPlayerController *)pController;
}

IPhysicsMotionController *CPhysicsEnvironment::CreateMotionController(IMotionEvent *pHandler) {
	CPhysicsMotionController *pController = (CPhysicsMotionController *)::CreateMotionController(this, pHandler);
	if (pController)
		m_controllers.AddToTail(pController);

	return pController;
}

void CPhysicsEnvironment::DestroyMotionController(IPhysicsMotionController *pController) {
	if (!pController) return;

	m_controllers.FindAndRemove((CPhysicsMotionController *)pController);
	delete (CPhysicsMotionController *)pController;
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

void SerializeWorld_f(const CCommand &args) {
	if (args.ArgC() != 2) {
		Msg("Usage: vphysics_serialize <index>\n");
		return;
	}

	CPhysicsEnvironment *pEnv = (CPhysicsEnvironment *)g_Physics.GetActiveEnvironmentByIndex(atoi(args.Arg(2)));
	if (pEnv) {
		btDiscreteDynamicsWorld *pWorld = (btDiscreteDynamicsWorld *)pEnv->GetBulletEnvironment();
		if (!pWorld) return;

		btSerializer *pSerializer = new btDefaultSerializer;
		pWorld->serialize(pSerializer);

		FILE *pFile = fopen("testfile.bullet", "wb");
		if (pFile) {
			fwrite(pSerializer->getBufferPointer(), pSerializer->getCurrentBufferSize(), 1, pFile);
			fclose(pFile);
		} else {
			Warning("Couldn't open testfile.bullet for writing!\n");
		}
	}
}

static ConCommand cmd_serializeworld("vphysics_serialize", SerializeWorld_f, "Serialize environment by index (usually 0=server, 1=client)\n\tDumps \"testfile.bullet\" out to the exe directory.");

static ConVar cvar_maxsubsteps("vphysics_maxsubsteps", "4", 0, "Sets the maximum amount of simulation substeps (higher number means higher precision)", true, 1, true, 150);
void CPhysicsEnvironment::Simulate(float deltaTime) {
	VPROF_BUDGET("CPhysicsEnvironment::Simulate", VPROF_BUDGETGROUP_PHYSICS);
	if (!m_pBulletEnvironment) Assert(0);

	if (deltaTime > 1.0 || deltaTime < 0.0) {
		deltaTime = 0;
	} else if (deltaTime > 0.1) {
		deltaTime = 0.1f;
	}

	//m_simPSIs = 0;
	//FIXME: figure out simulation time
	//m_simPSIcurrent = m_simPSIs;
	
	if (deltaTime > 1e-4) {
		m_inSimulation = true;

		// We're scaling the timestep down by the number of substeps to have a higher precision and take
		// the same amount of time as a simulation with the requested timestep
		float timestep = cvar_maxsubsteps.GetInt() != 0 ? m_timestep / cvar_maxsubsteps.GetInt() : m_timestep;
		
		VPROF_ENTER_SCOPE("m_pBulletEnvironment->stepSimulation");
		m_pBulletEnvironment->stepSimulation(deltaTime, cvar_maxsubsteps.GetInt(), timestep);
		VPROF_EXIT_SCOPE();

		m_inSimulation = false;
	}

#if DEBUG_DRAW
	VPROF_ENTER_SCOPE("m_debugdraw->DrawWorld");
	m_debugdraw->DrawWorld();
	VPROF_EXIT_SCOPE();
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

const IPhysicsObject **CPhysicsEnvironment::GetObjectList(int *pOutputObjectCount) const {
	if (pOutputObjectCount) {
		*pOutputObjectCount = m_objects.Count();
	}

	return (const IPhysicsObject **)m_objects.Base();
}

// TODO: This function may need some more work such as transferring objects with shadow controllers or fluid controllers
bool CPhysicsEnvironment::TransferObject(IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment) {
	if (!pObject || !pDestinationEnvironment) return false;

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

void CPhysicsEnvironment::CleanupDeleteList() {
	for (int i = 0; i < m_deadObjects.Count(); i++) {
		CPhysicsObject *pObject = (CPhysicsObject *)m_deadObjects.Element(i);
		delete pObject;
		pObject = NULL;
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
	if (!ray.m_IsRay || !pTrace) return;

	btVector3 vecStart, vecEnd;
	ConvertPosToBull(ray.m_Start + ray.m_StartOffset, vecStart);
	ConvertPosToBull(ray.m_Start + ray.m_StartOffset + ray.m_Delta, vecEnd);

	// TODO: Override this class to use the mask and trace filter.
	btCollisionWorld::ClosestRayResultCallback cb(vecStart, vecEnd);
	m_pBulletEnvironment->rayTest(vecStart, vecEnd, cb);

	pTrace->fraction = cb.m_closestHitFraction;
	ConvertPosToHL(cb.m_hitPointWorld, pTrace->endpos);
	ConvertDirectionToHL(cb.m_hitNormalWorld, pTrace->plane.normal);
}

// Is this function ever called?
// TODO: This is a bit more complex, bullet doesn't support compound sweep tests.
void CPhysicsEnvironment::SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	NOT_IMPLEMENTED
}

void CPhysicsEnvironment::SweepConvex(const CPhysConvex *pConvex, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	if (!pConvex || !pTrace) return;
	
	btVector3 vecStart, vecEnd;
	ConvertPosToBull(vecAbsStart, vecStart);
	ConvertPosToBull(vecAbsEnd, vecEnd);

	btMatrix3x3 matAng;
	ConvertRotationToBull(vecAngles, matAng);

	btTransform transStart, transEnd;
	transStart.setOrigin(vecStart);
	transStart.setBasis(matAng);

	transEnd.setOrigin(vecEnd);
	transEnd.setBasis(matAng);

	btConvexShape *pShape = (btConvexShape *)pConvex;

	btCollisionWorld::ClosestConvexResultCallback cb(vecStart, vecEnd);
	m_pBulletEnvironment->convexSweepTest(pShape, transStart, transEnd, cb, 0.0001f);

	pTrace->fraction = cb.m_closestHitFraction;
	ConvertPosToHL(cb.m_hitPointWorld, pTrace->endpos);
	ConvertDirectionToHL(cb.m_hitNormalWorld, pTrace->plane.normal);
}

void CPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t *pOutput) const {
	if (!pOutput) return;

	*pOutput = m_perfparams;
}

void CPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t *pSettings) {
	if (!pSettings) return;

	m_perfparams = *pSettings;
}

void CPhysicsEnvironment::ReadStats(physics_stats_t *pOutput) {
	if (!pOutput) return;

	*pOutput = m_stats;
}

void CPhysicsEnvironment::ClearStats() {
	memset(&m_stats, 0, sizeof(m_stats));
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
	m_bConstraintNotify = bEnable;
}

// FIXME: What do? Source SDK mods call this every frame in debug builds.
void CPhysicsEnvironment::DebugCheckContacts() {
	
}

// UNEXPOSED
btSoftRigidDynamicsWorld *CPhysicsEnvironment::GetBulletEnvironment() {
	return m_pBulletEnvironment;
}

// UNEXPOSED
float CPhysicsEnvironment::GetInvPSIScale() {
	//FIXME: get correct value
	//return m_invPSIscale;
	return 0.5;
}

// UNEXPOSED
void CPhysicsEnvironment::BulletTick(btScalar dt) {
	VPROF_BUDGET("CPhysicsEnvironment::BulletTick", VPROF_BUDGETGROUP_PHYSICS);
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

// UNEXPOSED
CPhysicsDragController *CPhysicsEnvironment::GetDragController() {
	return m_pPhysicsDragController;
}

// UNEXPOSED
// Purpose: To be the biggest eyesore ever
// Bullet doesn't provide many callbacks such as the ones we're looking for, so
// we have to iterate through all the contact manifolds and generate the callbacks ourselves.
void CPhysicsEnvironment::DoCollisionEvents(float dt) {
	VPROF_BUDGET("CPhysicsEnvironment::DoCollisionEvents", VPROF_BUDGETGROUP_PHYSICS);

	if (m_pCollisionEvent) {
		int numManifolds = m_pBulletEnvironment->getDispatcher()->getNumManifolds();
		for (int i = 0; i < numManifolds; i++) {
			btPersistentManifold *contactManifold = m_pBulletEnvironment->getDispatcher()->getManifoldByIndexInternal(i);
			if (contactManifold->getNumContacts() <= 0)
				continue;

			// obA is the object which is colliding with obB
			const btCollisionObject *obA = contactManifold->getBody0();
			const btCollisionObject *obB = contactManifold->getBody1();
			if (!obA || !obB) continue;

			// These are our own internal objects, don't do callbacks on them.
			if (obA->getInternalType() == btCollisionObject::CO_GHOST_OBJECT || obB->getInternalType() == btCollisionObject::CO_GHOST_OBJECT)
				continue;

			for (int j = 0; j < contactManifold->getNumContacts(); j++) {
				btManifoldPoint manPoint = contactManifold->getContactPoint(j);
				
				// FRICTION CALLBACK
				if (((CPhysicsObject *)obA->getUserPointer())->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION) {
					// FIXME: We need to find the energy used by the friction! Bullet doesn't provide this in the manifold point.
					// This may not be the proper variable but whatever, as of now it works.
					float energy = abs(manPoint.m_appliedImpulseLateral1);
					if (energy > 0.05f) {
						CPhysicsCollisionData data(&manPoint);
						m_pCollisionEvent->Friction((CPhysicsObject *)obA->getUserPointer(),
													ConvertEnergyToHL(energy),
													((CPhysicsObject *)obA->getUserPointer())->GetMaterialIndex(),
													((CPhysicsObject *)obB->getUserPointer())->GetMaterialIndex(),
													&data);
					}
				}

				// TODO: Collision callback
				// Source wants precollision and postcollision callbacks (pre velocity and post velocity, etc.)
				// How do we generate a callback before the collision happens?
			}
		}
	}

	if (m_pObjectEvent) {
		int numObjects = m_pBulletEnvironment->getNumCollisionObjects();
		btCollisionObjectArray collisionObjects = m_pBulletEnvironment->getCollisionObjectArray();
		for (int i = 0; i < numObjects; i++) {
			btCollisionObject *obj = collisionObjects[i];
			CPhysicsObject *physobj = (CPhysicsObject *)collisionObjects[i]->getUserPointer();

			if (physobj->GetLastActivationState() != obj->getActivationState()) {
				switch (obj->getActivationState()) {
					case ACTIVE_TAG:
						m_pObjectEvent->ObjectWake(physobj);
						break;
					case ISLAND_SLEEPING:
						m_pObjectEvent->ObjectSleep(physobj);
						break;
				}

				physobj->SetLastActivationState(obj->getActivationState());
			}
		}
	}
}

// UNEXPOSED
void CPhysicsEnvironment::HandleConstraintBroken(CPhysicsConstraint *pConstraint) {
	if (m_bConstraintNotify && m_pConstraintEvent)
		m_pConstraintEvent->ConstraintBroken(pConstraint);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleFluidStartTouch(CPhysicsFluidController *pController, CPhysicsObject *pObject) {
	if (m_pCollisionEvent)
		m_pCollisionEvent->FluidStartTouch(pObject, pController);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleFluidEndTouch(CPhysicsFluidController *pController, CPhysicsObject *pObject) {
	if (m_pCollisionEvent)
		m_pCollisionEvent->FluidEndTouch(pObject, pController);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleObjectEnteredTrigger(CPhysicsObject *pTrigger, CPhysicsObject *pObject) {
	if (m_pCollisionEvent)
		m_pCollisionEvent->ObjectEnterTrigger(pTrigger, pObject);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleObjectExitedTrigger(CPhysicsObject *pTrigger, CPhysicsObject *pObject) {
	if (m_pCollisionEvent)
		m_pCollisionEvent->ObjectLeaveTrigger(pTrigger, pObject);
}