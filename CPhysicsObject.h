#ifndef CPHYSICSOBJECT_H
#define CPHYSICSOBJECT_H

class CPhysicsEnvironment;
class CShadowController;
class CPhysicsFluidController;
class CPhysicsConstraint;

class CPhysicsObject : public IPhysicsObject {
public:
	CPhysicsObject();
	~CPhysicsObject();
	virtual bool IsStatic() const;
	virtual bool IsAsleep() const;
	virtual bool IsTrigger() const;
	virtual bool IsFluid() const;
	virtual bool IsHinged() const;
	virtual bool IsCollisionEnabled() const;
	virtual bool IsGravityEnabled() const;
	virtual bool IsDragEnabled() const;
	virtual bool IsMotionEnabled() const;
	virtual bool IsMoveable() const;
	virtual bool IsAttachedToConstraint(bool bExternalOnly) const;

	virtual void EnableCollisions(bool enable);
	virtual void EnableGravity(bool enable);
	virtual void EnableDrag(bool enable);
	virtual void EnableMotion(bool enable);

	virtual void SetGameData(void* pGameData);
	virtual void* GetGameData() const;
	virtual void SetGameFlags(unsigned short userFlags);
	virtual unsigned short GetGameFlags() const;
	virtual void SetGameIndex(unsigned short gameIndex);
	virtual unsigned short GetGameIndex() const;
	virtual void SetCallbackFlags(unsigned short callbackflags);
	virtual unsigned short GetCallbackFlags() const;

	virtual void Wake();
	virtual void Sleep();

	virtual void RecheckCollisionFilter();
	virtual void RecheckContactPoints();

	virtual void SetMass(float mass);
	virtual float GetMass() const;
	virtual float GetInvMass() const;
	virtual Vector GetInertia() const;
	virtual Vector GetInvInertia() const;
	virtual void SetInertia(const Vector& inertia);
	virtual void SetDamping(const float* speed, const float* rot);
	virtual void GetDamping(float* speed, float* rot) const;
	virtual void SetDragCoefficient(float* pDrag, float* pAngularDrag);
	virtual void SetBuoyancyRatio(float ratio);
	virtual int GetMaterialIndex() const;
	virtual void SetMaterialIndex(int materialIndex);
	virtual unsigned int GetContents() const;
	virtual void SetContents(unsigned int contents);
	virtual float GetSphereRadius() const;
	virtual float GetEnergy() const;
	virtual Vector GetMassCenterLocalSpace() const;

	virtual void SetPosition(const Vector& worldPosition, const QAngle& angles, bool isTeleport);
	virtual void SetPositionMatrix(const matrix3x4_t&matrix, bool isTeleport);
	virtual void GetPosition(Vector* worldPosition, QAngle* angles) const;
	virtual void GetPositionMatrix(matrix3x4_t* positionMatrix) const;

	virtual void SetVelocity(const Vector* velocity, const AngularImpulse* angularVelocity);
	virtual void SetVelocityInstantaneous(const Vector* velocity, const AngularImpulse* angularVelocity);
	virtual void GetVelocity(Vector* velocity, AngularImpulse* angularVelocity) const;
	virtual void AddVelocity(const Vector* velocity, const AngularImpulse* angularVelocity);
	virtual void GetVelocityAtPoint(const Vector& worldPosition, Vector* pVelocity) const;
	virtual void GetImplicitVelocity(Vector* velocity, AngularImpulse* angularVelocity) const;

	virtual void LocalToWorld(Vector* worldPosition, const Vector& localPosition) const;
	virtual void WorldToLocal(Vector* localPosition, const Vector& worldPosition) const;
	virtual void LocalToWorldVector(Vector* worldVector, const Vector& localVector) const;
	virtual void WorldToLocalVector(Vector* localVector, const Vector& worldVector) const;
	
	virtual void ApplyForceCenter(const Vector& forceVector);
	virtual void ApplyForceOffset(const Vector& forceVector, const Vector& worldPosition);
	virtual void ApplyTorqueCenter(const AngularImpulse& torque);

	virtual void CalculateForceOffset(const Vector& forceVector, const Vector& worldPosition, Vector* centerForce, AngularImpulse* centerTorque) const;
	virtual void CalculateVelocityOffset(const Vector& forceVector, const Vector& worldPosition, Vector* centerVelocity, AngularImpulse* centerAngularVelocity) const;
	virtual float CalculateLinearDrag(const Vector& unitDirection) const;
	virtual float CalculateAngularDrag(const Vector& objectSpaceRotationAxis) const;

	virtual bool GetContactPoint(Vector* contactPoint, IPhysicsObject* *contactObject) const;

	virtual void SetShadow(float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation);
	virtual void UpdateShadow(const Vector& targetPosition, const QAngle& targetAngles, bool tempDisableGravity, float timeOffset);
	
	virtual int GetShadowPosition(Vector* position, QAngle* angles) const;
	virtual IPhysicsShadowController* GetShadowController() const;
	virtual void RemoveShadowController();
	virtual float ComputeShadowControl(const hlshadowcontrol_params_t& params, float secondsToArrival, float dt);

	virtual const CPhysCollide* GetCollide() const;
	virtual const char* GetName() const;

	virtual void BecomeTrigger();
	virtual void RemoveTrigger();

	virtual void BecomeHinged(int localAxis);
	virtual void RemoveHinged();

	virtual IPhysicsFrictionSnapshot* CreateFrictionSnapshot();
	virtual void DestroyFrictionSnapshot(IPhysicsFrictionSnapshot* pSnapshot);

	virtual void OutputDebugInfo() const;

	virtual CPhysicsFluidController *GetFluidController(void) { return m_pFluidController; }
	virtual void SetFluidController(CPhysicsFluidController *controller) { m_pFluidController = controller; }
public:
	void Init(CPhysicsEnvironment* pEnv, btRigidBody* pObject, int materialIndex, float volume, float drag, float angDrag, const Vector *massCenterOverride);
	CPhysicsEnvironment* GetVPhysicsEnvironment();
	btRigidBody* GetObject();
	float GetDragInDirection(btVector3 * direction) const; // Function is not interfaced anymore
	float GetAngularDragInDirection(btVector3 * direction) const;

	int m_iLastActivationState;
private:
	CPhysicsEnvironment* m_pEnv;
	void* m_pGameData;
	btRigidBody* m_pObject;

	unsigned short m_materialIndex;

	unsigned short m_callbacks;
	unsigned short m_gameFlags;
	unsigned int m_contents;

	float m_volume;
	float m_dragCoefficient;
	float m_angDragCoefficient;
	btVector3 m_dragBasis;
	btVector3 m_angDragBasis;
	Vector m_massCenterOverride;
	CShadowController *m_pShadow;
	CPhysicsFluidController *m_pFluidController;
};

CPhysicsObject* CreatePhysicsObject(CPhysicsEnvironment* pEnvironment, const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams, bool isStatic);
CPhysicsObject* CreatePhysicsSphere(CPhysicsEnvironment *pEnvironment, float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic);

#endif