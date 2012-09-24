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

class btDistanceConstraint : public btPoint2PointConstraint
{
protected:
	btScalar	m_distance;
public:
	btDistanceConstraint(btRigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB, btScalar dist)
		: btPoint2PointConstraint(rbA, rbB, pivotInA, pivotInB)
	{
		m_distance = dist;
	}
	virtual void getInfo1 (btConstraintInfo1* info)
	{
		info->m_numConstraintRows = 1;
		info->nub = 5;
	}
	virtual void getInfo2 (btConstraintInfo2* info)
	{
		btVector3 relA = m_rbA.getCenterOfMassTransform().getBasis() * getPivotInA();
		btVector3 relB = m_rbB.getCenterOfMassTransform().getBasis() * getPivotInB();
		btVector3 posA = m_rbA.getCenterOfMassTransform().getOrigin() + relA;
		btVector3 posB = m_rbB.getCenterOfMassTransform().getOrigin() + relB;
		btVector3 del = posB - posA;
		btScalar currDist = btSqrt(del.dot(del));
		btVector3 ortho = del / currDist;
		info->m_J1linearAxis[0] = ortho[0];
		info->m_J1linearAxis[1] = ortho[1];
		info->m_J1linearAxis[2] = ortho[2];
		btVector3 p, q;
		p = relA.cross(ortho);
		q = relB.cross(ortho);
		info->m_J1angularAxis[0] = p[0];
		info->m_J1angularAxis[1] = p[1];
		info->m_J1angularAxis[2] = p[2];
		info->m_J2angularAxis[0] = -q[0];
		info->m_J2angularAxis[1] = -q[1];
		info->m_J2angularAxis[2] = -q[2];
		btScalar rhs = (currDist - m_distance) * info->fps * info->erp;
		info->m_constraintError[0] = rhs;
		info->cfm[0] = btScalar(0.f);
		info->m_lowerLimit[0] = -SIMD_INFINITY;
		info->m_upperLimit[0] = SIMD_INFINITY;
	}
};

// FIXME: I dont think we can implement this in Bullet anyways?
class CPhysicsConstraintGroup : public IPhysicsConstraintGroup
{
public:
	~CPhysicsConstraintGroup(void) {}
	void Activate() { NOT_IMPLEMENTED; };
	bool IsInErrorState() { return false; };
	void ClearErrorState() { };
	void GetErrorParams(constraint_groupparams_t *pParams) { NOT_IMPLEMENTED; };
	void SetErrorParams(const constraint_groupparams_t &params) { NOT_IMPLEMENTED; };
	void SolvePenetration(IPhysicsObject *pObj0, IPhysicsObject *pObj1) { NOT_IMPLEMENTED; };
};

#endif
