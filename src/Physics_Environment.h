#ifndef CPHYSICSENVIRONMENT_H
#define CPHYSICSENVIRONMENT_H

#include <vphysics/performance.h>
#include <vphysics/stats.h>

class btThreadSupportInterface;
class btCollisionConfiguration;
class btDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btSoftRigidDynamicsWorld;

class IController;
class CDeleteQueue;
class CCollisionSolver;
class CPhysicsFluidController;
class CPhysicsDragController;
class CPhysicsConstraint;
class CPhysicsObject;

class CDebugDrawer;

class CPhysicsEnvironment : public IPhysicsEnvironment {
public:
											CPhysicsEnvironment();
											~CPhysicsEnvironment();

	// UNEXPOSED
	// Don't call this directly!
	static void								TickCallback(btDynamicsWorld *world, btScalar timestep);

	void									SetDebugOverlay(CreateInterfaceFn debugOverlayFactory);
	IVPhysicsDebugOverlay *					GetDebugOverlay();
	btIDebugDraw *							GetDebugDrawer();

	void									SetGravity(const Vector& gravityVector);
	void									GetGravity(Vector* pGravityVector) const;

	void									SetAirDensity(float density);
	float									GetAirDensity() const;
	
	IPhysicsObject *						CreatePolyObject(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams);
	IPhysicsObject *						CreatePolyObjectStatic(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams);
	IPhysicsObject *						CreateSphereObject(float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic = false);
	void									DestroyObject(IPhysicsObject*);

	IPhysicsFluidController	*				CreateFluidController(IPhysicsObject *pFluidObject, fluidparams_t *pParams);
	void									DestroyFluidController(IPhysicsFluidController*);

	IPhysicsSpring	*						CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams);
	void									DestroySpring(IPhysicsSpring*);

	IPhysicsConstraint *					CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll);
	IPhysicsConstraint *					CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge);
	IPhysicsConstraint *					CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed);
	IPhysicsConstraint *					CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding);
	IPhysicsConstraint *					CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket);
	IPhysicsConstraint *					CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley);
	IPhysicsConstraint *					CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length);

	void									DestroyConstraint(IPhysicsConstraint *pConstraint);

	IPhysicsConstraintGroup *				CreateConstraintGroup(const constraint_groupparams_t &groupParams);
	void									DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup);

	IPhysicsShadowController *				CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation);
	void									DestroyShadowController(IPhysicsShadowController *pController);

	IPhysicsPlayerController *				CreatePlayerController(IPhysicsObject *pObject);
	void									DestroyPlayerController(IPhysicsPlayerController *pController);

	IPhysicsMotionController *				CreateMotionController(IMotionEvent *pHandler);
	void									DestroyMotionController(IPhysicsMotionController *pController);

	IPhysicsVehicleController *				CreateVehicleController(IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace);
	void									DestroyVehicleController(IPhysicsVehicleController *pController);

	void									SetCollisionSolver(IPhysicsCollisionSolver *pSolver);

	void									Simulate(float deltaTime);
	bool									IsInSimulation() const;

	float									GetSimulationTimestep() const;
	void									SetSimulationTimestep(float timestep);

	float									GetSimulationTime() const;
	void									ResetSimulationClock();

	float									GetNextFrameTime() const;

	void									SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents);
	void									SetObjectEventHandler(IPhysicsObjectEvent *pObjectEvents);
	void									SetConstraintEventHandler(IPhysicsConstraintEvent *pConstraintEvents);

	void									SetQuickDelete(bool bQuick);

	int										GetActiveObjectCount() const;
	void									GetActiveObjects(IPhysicsObject **pOutputObjectList) const;
	const IPhysicsObject **					GetObjectList(int *pOutputObjectCount) const;
	bool									TransferObject(IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment);

	void									CleanupDeleteList(void);
	void									EnableDeleteQueue(bool enable);

	bool									Save(const physsaveparams_t &params);
	void									PreRestore(const physprerestoreparams_t &params);
	bool									Restore(const physrestoreparams_t &params);
	void									PostRestore();

	bool									IsCollisionModelUsed(CPhysCollide *pCollide) const;
	
	void									TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);
	void									SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);
	void									SweepConvex(const CPhysConvex *pConvex, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);

	void									GetPerformanceSettings(physics_performanceparams_t *pOutput) const;
	void									SetPerformanceSettings(const physics_performanceparams_t *pSettings);

	void									ReadStats(physics_stats_t *pOutput);
	void									ClearStats();

	unsigned int							GetObjectSerializeSize(IPhysicsObject *pObject) const;
	void									SerializeObjectToBuffer(IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize);
	IPhysicsObject *						UnserializeObjectFromBuffer(void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions);

	void									EnableConstraintNotify(bool bEnable);
	void									DebugCheckContacts();
public:
	// Unexposed functions
	btDynamicsWorld *						GetBulletEnvironment();
	float									GetInvPSIScale();
	void									BulletTick(btScalar timeStep);
	CPhysicsDragController *				GetDragController();

	void									DoCollisionEvents(float dt);

	void									HandleConstraintBroken(CPhysicsConstraint *pConstraint); // Call this if you're a constraint that was just broken.
	void									HandleFluidStartTouch(CPhysicsFluidController *pController, CPhysicsObject *pObject);
	void									HandleFluidEndTouch(CPhysicsFluidController *pController, CPhysicsObject *pObject);
	void									HandleObjectEnteredTrigger(CPhysicsObject *pTrigger, CPhysicsObject *pObject);
	void									HandleObjectExitedTrigger(CPhysicsObject *pTrigger, CPhysicsObject *pObject);

	// Soft body functions we'll expose at a later time...
private:
	bool									m_inSimulation;
	bool									m_bUseDeleteQueue;
	bool									m_bConstraintNotify;
	bool									m_deleteQuick;
	float									m_timestep;

	btThreadSupportInterface *				m_pThreadSupportCollision;
	btThreadSupportInterface *				m_pThreadSupportSolver;
	btCollisionConfiguration *				m_pBulletConfiguration;
	btCollisionDispatcher *					m_pBulletDispatcher;
	btBroadphaseInterface *					m_pBulletBroadphase;
	btConstraintSolver *					m_pBulletSolver;
	btSoftRigidDynamicsWorld *				m_pBulletEnvironment;
	btOverlappingPairCallback *				m_pBulletGhostCallback;

	CUtlVector<IPhysicsObject *>			m_objects;
	CUtlVector<IPhysicsObject *>			m_deadObjects;
	CCollisionSolver *						m_pCollisionSolver;
	CDeleteQueue *							m_pDeleteQueue;
	CUtlVector<CPhysicsFluidController *>	m_fluids;
	CPhysicsDragController *				m_pPhysicsDragController;
	CUtlVector<IController*>				m_controllers;
	IVPhysicsDebugOverlay *					m_pDebugOverlay;

	IPhysicsCollisionEvent *				m_pCollisionEvent;
	IPhysicsConstraintEvent *				m_pConstraintEvent;
	IPhysicsObjectEvent *					m_pObjectEvent;

	physics_performanceparams_t				m_perfparams;
	physics_stats_t							m_stats;

	CDebugDrawer *							m_debugdraw;
};

#endif