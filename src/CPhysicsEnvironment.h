#ifndef CPHYSICSENVIRONMENT_H
#define CPHYSICSENVIRONMENT_H

class btThreadSupportInterface;
class btCollisionConfiguration;
class btDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btDynamicsWorld;
class IController;
class CDeleteQueue;
class CCollisionSolver;
class CPhysicsFluidController;
class CPhysicsDragController;

class CDebugDrawer;

class CPhysicsEnvironment : public IPhysicsEnvironment {
public:
											CPhysicsEnvironment();
											~CPhysicsEnvironment();

	void									SetDebugOverlay(CreateInterfaceFn debugOverlayFactory);
	IVPhysicsDebugOverlay *					GetDebugOverlay();

	void									SetGravity(const Vector& gravityVector);
	void									GetGravity(Vector* pGravityVector) const;

	void									SetAirDensity(float density);
	float									GetAirDensity(void) const;
	
	IPhysicsObject *						CreatePolyObject(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams);
	IPhysicsObject *						CreatePolyObjectStatic(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams);
	IPhysicsObject *						CreateSphereObject(float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic);
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

	void									DestroyConstraint(IPhysicsConstraint*);

	IPhysicsConstraintGroup *				CreateConstraintGroup(const constraint_groupparams_t &groupParams);
	void									DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup);

	IPhysicsShadowController *				CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation);
	void									DestroyShadowController(IPhysicsShadowController*);

	IPhysicsPlayerController *				CreatePlayerController(IPhysicsObject* pObject);
	void									DestroyPlayerController(IPhysicsPlayerController*);

	IPhysicsMotionController *				CreateMotionController(IMotionEvent *pHandler);
	void									DestroyMotionController(IPhysicsMotionController *pController);

	IPhysicsVehicleController *				CreateVehicleController(IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace);
	void									DestroyVehicleController(IPhysicsVehicleController*);

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
	void									GetActiveObjects(IPhysicsObject **pOutputObjectList ) const;
	const IPhysicsObject **					GetObjectList(int *pOutputObjectCount ) const;
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

	void									GetPerformanceSettings(physics_performanceparams_t *pOutput) const;
	void									SetPerformanceSettings(const physics_performanceparams_t *pSettings);

	void									ReadStats(physics_stats_t *pOutput);
	void									ClearStats();

	unsigned int							GetObjectSerializeSize(IPhysicsObject *pObject) const;
	void									SerializeObjectToBuffer(IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize);
	IPhysicsObject *						UnserializeObjectFromBuffer(void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions);

	void									EnableConstraintNotify(bool bEnable);
	void									DebugCheckContacts(void);

	// UNEXPOSED
	void									DoCollisionEvents();
public:
	btDynamicsWorld *						GetBulletEnvironment();
	float									GetInvPSIScale();
	void									BulletTick(btScalar timeStep);
	CPhysicsDragController *				GetDragController();
	IPhysicsObjectEvent *					m_pObjectEvent;
private:
	bool									m_inSimulation;
	bool									m_bUseDeleteQueue;
	btThreadSupportInterface *				m_pThreadSupportCollision;
	btThreadSupportInterface *				m_pThreadSupportSolver;
	btCollisionConfiguration *				m_pBulletConfiguration;
	btCollisionDispatcher *					m_pBulletDispatcher;
	btBroadphaseInterface *					m_pBulletBroadphase;
	btConstraintSolver *					m_pBulletSolver;
	btDynamicsWorld *						m_pBulletEnvironment;
	CUtlVector<IPhysicsObject*>				m_objects;
	CUtlVector<IPhysicsObject*>				m_deadObjects;
	CCollisionSolver *						m_pCollisionSolver;
	IPhysicsCollisionEvent *				m_pCollisionEvent;
	IPhysicsConstraintEvent *				m_pConstraintEvent;
	CDeleteQueue *							m_pDeleteQueue;
	physics_performanceparams_t *			m_physics_performanceparams;
	CUtlVector<CPhysicsFluidController*>	m_fluids;
	CPhysicsDragController *				m_pPhysicsDragController;
	CUtlVector<IController*>				m_controllers;
	bool									m_deleteQuick;
	float									m_timestep;
	IVPhysicsDebugOverlay *					m_pDebugOverlay;

	CDebugDrawer *							m_debugdraw;
};

#endif