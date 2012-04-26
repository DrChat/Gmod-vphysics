#ifndef CCONSTRAINT_H
#define CCONSTRAINT_H

class CPhysicsEnvironment;

class CPhysicsConstraint : public IPhysicsConstraint {
public:
	CPhysicsConstraint(CPhysicsEnvironment *pEnv, CPhysicsObject* pObject1, CPhysicsObject* pObject2, btTypedConstraint *pConstraint);
	~CPhysicsConstraint();

	virtual void Activate();
	virtual void Deactivate();

	virtual void SetGameData(void *gameData) { m_pGameData = gameData; };
	virtual void *GetGameData(void) const { return m_pGameData; };

	virtual IPhysicsObject *GetReferenceObject(void) const { return m_pObject2; };
	virtual IPhysicsObject *GetAttachedObject(void) const { return m_pObject1; };

	// What the christ are these?
	virtual void			SetLinearMotor(float speed, float maxLinearImpulse) { NOT_IMPLEMENTED; };
	virtual void			SetAngularMotor(float rotSpeed, float maxAngularImpulse) { NOT_IMPLEMENTED; };

	virtual void			UpdateRagdollTransforms(const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached) { NOT_IMPLEMENTED; };
	virtual bool			GetConstraintTransform(matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached) const { NOT_IMPLEMENTED; return false; };
	virtual bool			GetConstraintParams(constraint_breakableparams_t *pParams) const { NOT_IMPLEMENTED; return false; };

	virtual void			OutputDebugInfo() { NOT_IMPLEMENTED; };

private:
	CPhysicsObject *m_pObject1;
	CPhysicsObject *m_pObject2;
	btTypedConstraint *m_pConstraint;
	void *m_pGameData;
	CPhysicsEnvironment *m_pEnv;
};

#endif
