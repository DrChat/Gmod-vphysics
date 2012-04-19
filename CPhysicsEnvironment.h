#ifndef CPHYSICSENVIRONMENT_H
#define CPHYSICSENVIRONMENT_H

class btCollisionConfiguration;
class btDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btDynamicsWorld;
class CDeleteQueue;
class CCollisionSolver;
class CPhysicsFluidController;

class CPhysicsEnvironment : public IPhysicsEnvironment {
public:
	CPhysicsEnvironment();
	virtual ~CPhysicsEnvironment();

	virtual void SetDebugOverlay(CreateInterfaceFn debugOverlayFactory);
	virtual IVPhysicsDebugOverlay *GetDebugOverlay();

	virtual void SetGravity(const Vector& gravityVector);
	virtual void GetGravity(Vector* pGravityVector) const;

	virtual void SetAirDensity(float density);
	virtual float GetAirDensity(void) const;
	
	virtual IPhysicsObject *CreatePolyObject(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams);
	virtual IPhysicsObject *CreatePolyObjectStatic(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams);
	virtual IPhysicsObject *CreateSphereObject(float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic);
	virtual void DestroyObject(IPhysicsObject*);

	virtual IPhysicsFluidController	*CreateFluidController(IPhysicsObject *pFluidObject, fluidparams_t *pParams);
	virtual void DestroyFluidController(IPhysicsFluidController*);

	virtual IPhysicsSpring	*CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams);
	virtual void DestroySpring(IPhysicsSpring*);

	virtual IPhysicsConstraint *CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll);
	virtual IPhysicsConstraint *CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge);
	virtual IPhysicsConstraint *CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed);
	virtual IPhysicsConstraint *CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding);
	virtual IPhysicsConstraint *CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket);
	virtual IPhysicsConstraint *CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley);
	virtual IPhysicsConstraint *CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length);

	virtual void DestroyConstraint(IPhysicsConstraint*);

	virtual IPhysicsConstraintGroup *CreateConstraintGroup(const constraint_groupparams_t &groupParams);
	virtual void DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup);

	virtual IPhysicsShadowController *CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation);
	virtual void DestroyShadowController(IPhysicsShadowController*);

	virtual IPhysicsPlayerController *CreatePlayerController(IPhysicsObject* pObject);
	virtual void DestroyPlayerController(IPhysicsPlayerController*);

	virtual IPhysicsMotionController *CreateMotionController(IMotionEvent *pHandler);
	virtual void DestroyMotionController(IPhysicsMotionController *pController);

	virtual IPhysicsVehicleController *CreateVehicleController(IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace);
	virtual void DestroyVehicleController(IPhysicsVehicleController*);

	virtual void SetCollisionSolver(IPhysicsCollisionSolver *pSolver);

	virtual void Simulate(float deltaTime);
	virtual bool IsInSimulation() const;

	virtual float GetSimulationTimestep() const;
	virtual void SetSimulationTimestep(float timestep);

	virtual float GetSimulationTime() const;
	virtual void ResetSimulationClock();

	virtual float GetNextFrameTime() const;

	virtual void SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents);
	virtual void SetObjectEventHandler(IPhysicsObjectEvent *pObjectEvents);
	virtual	void SetConstraintEventHandler(IPhysicsConstraintEvent *pConstraintEvents);

	virtual void SetQuickDelete(bool bQuick);

	virtual int GetActiveObjectCount() const;
	virtual void GetActiveObjects(IPhysicsObject **pOutputObjectList ) const;
	virtual const IPhysicsObject **GetObjectList(int *pOutputObjectCount ) const;
	virtual bool TransferObject(IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment);

	virtual void CleanupDeleteList(void);
	virtual void EnableDeleteQueue(bool enable);

	virtual bool Save(const physsaveparams_t &params);
	virtual void PreRestore(const physprerestoreparams_t &params);
	virtual bool Restore(const physrestoreparams_t &params);
	virtual void PostRestore();

	virtual bool IsCollisionModelUsed(CPhysCollide *pCollide) const;
	
	virtual void TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);
	virtual void SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);

	virtual void GetPerformanceSettings(physics_performanceparams_t *pOutput) const;
	virtual void SetPerformanceSettings(const physics_performanceparams_t *pSettings);

	virtual void ReadStats(physics_stats_t *pOutput);
	virtual void ClearStats();

	virtual unsigned int GetObjectSerializeSize(IPhysicsObject *pObject) const;
	virtual void SerializeObjectToBuffer(IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize);
	virtual IPhysicsObject *UnserializeObjectFromBuffer(void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions);

	virtual void EnableConstraintNotify(bool bEnable);
	virtual void DebugCheckContacts(void);
public:
	btDynamicsWorld* GetBulletEnvironment();
private:
	bool m_inSimulation;
	bool m_queueDeleteObject;
	btCollisionConfiguration* m_pBulletConfiguration;
	btDispatcher* m_pBulletDispatcher;
	btBroadphaseInterface* m_pBulletBroadphase;
	btConstraintSolver* m_pBulletSolver;
	btDynamicsWorld* m_pBulletEnvironment;
	CUtlVector<IPhysicsObject*> m_objects;
	CUtlVector<IPhysicsObject*> m_deadObjects;
	CCollisionSolver* m_pCollisionSolver;
	IPhysicsCollisionEvent* m_pCollisionEvent;
	IPhysicsObjectEvent* m_pObjectEvent;
	IPhysicsConstraintEvent* m_pConstraintEvent;
	CDeleteQueue *m_pDeleteQueue;
	physics_performanceparams_t * m_physics_peformanceparams;
	CUtlVector<CPhysicsFluidController *> m_fluids;
	bool m_deleteQuick;
	float m_timestep;
};

#endif