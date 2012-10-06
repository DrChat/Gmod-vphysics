#ifndef CCONSTRAINT_H
#define CCONSTRAINT_H

class CPhysicsEnvironment;

enum EConstraintType {
	CONSTRAINT_RAGDOLL,
	CONSTRAINT_HINGE,
	CONSTRAINT_FIXED,
	CONSTRAINT_SLIDING,
	CONSTRAINT_BALLSOCKET,
	CONSTRAINT_PULLEY,
	CONSTRAINT_LENGTH
};

class CPhysicsConstraint : public IPhysicsConstraint {
	public:
								CPhysicsConstraint(CPhysicsEnvironment *pEnv, CPhysicsObject* pObject1, CPhysicsObject* pObject2, btTypedConstraint *pConstraint, EConstraintType type);
								~CPhysicsConstraint();

		void					Activate();
		void					Deactivate();

		void					SetGameData(void *gameData) { m_pGameData = gameData; };
		void *					GetGameData(void) const { return m_pGameData; };

		IPhysicsObject *		GetReferenceObject(void) const { return m_pObject2; };
		IPhysicsObject *		GetAttachedObject(void) const { return m_pObject1; };

		void					SetLinearMotor(float speed, float maxLinearImpulse);
		void					SetAngularMotor(float rotSpeed, float maxAngularImpulse);
		
		void					UpdateRagdollTransforms(const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached);
		bool					GetConstraintTransform(matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached) const;
		bool					GetConstraintParams(constraint_breakableparams_t *pParams) const;
		
		void					OutputDebugInfo();

	private:
		CPhysicsObject *		m_pObject1;	// Reference object
		CPhysicsObject *		m_pObject2;	// Attached object
		btTypedConstraint *		m_pConstraint;
		void *					m_pGameData;
		CPhysicsEnvironment *	m_pEnv;
		EConstraintType			m_type;
};

// FIXME: I dont think we can implement this in Bullet anyways?
class CPhysicsConstraintGroup : public IPhysicsConstraintGroup
{
	public:
				~CPhysicsConstraintGroup(void) {}
		void	Activate() { NOT_IMPLEMENTED; };
		bool	IsInErrorState() { return false; };
		void	ClearErrorState() { };
		void	GetErrorParams(constraint_groupparams_t *pParams) { NOT_IMPLEMENTED; };
		void	SetErrorParams(const constraint_groupparams_t &params) { NOT_IMPLEMENTED; };
		void	SolvePenetration(IPhysicsObject *pObj0, IPhysicsObject *pObj1) { NOT_IMPLEMENTED; };
};

// CONSTRAINT CREATION FUNCTIONS
CPhysicsConstraint *CreateRagdollConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll);
CPhysicsConstraint *CreateHingeConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge);
CPhysicsConstraint *CreateFixedConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed);
CPhysicsConstraint *CreateSlidingConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding);
CPhysicsConstraint *CreateBallsocketConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket);
CPhysicsConstraint *CreatePulleyConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley);
CPhysicsConstraint *CreateLengthConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length);

#endif