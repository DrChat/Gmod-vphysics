#ifndef CCONSTRAINT_H
#define CCONSTRAINT_H

class CPhysicsEnvironment;

enum EConstraintType {
	CONSTRAINT_UNKNOWN,
	CONSTRAINT_RAGDOLL,
	CONSTRAINT_HINGE,
	CONSTRAINT_FIXED,
	CONSTRAINT_SLIDING,
	CONSTRAINT_BALLSOCKET,
	CONSTRAINT_PULLEY,
	CONSTRAINT_LENGTH,
	CONSTRAINT_SPRING
};

class CPhysicsConstraint : public IPhysicsConstraint {
	public:
		CPhysicsConstraint(CPhysicsEnvironment *pEnv, CPhysicsObject *pObject1, CPhysicsObject *pObject2, btTypedConstraint *pConstraint, EConstraintType type);
		~CPhysicsConstraint();

		void					Activate();
		void					Deactivate();

		void					SetGameData(void *gameData) { m_pGameData = gameData; };
		void *					GetGameData() const { return m_pGameData; };

		IPhysicsObject *		GetReferenceObject() const { return m_pReferenceObject; };
		IPhysicsObject *		GetAttachedObject() const { return m_pAttachedObject; };

		void					SetLinearMotor(float speed, float maxLinearImpulse);
		void					SetAngularMotor(float rotSpeed, float maxAngularImpulse);
		
		void					UpdateRagdollTransforms(const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached);
		bool					GetConstraintTransform(matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached) const;
		bool					GetConstraintParams(constraint_breakableparams_t *pParams) const;
		
		void					OutputDebugInfo();

		// UNEXPOSED FUNCTIONS
	protected:
		CPhysicsObject *		m_pReferenceObject;	// Reference object
		CPhysicsObject *		m_pAttachedObject;	// Attached object
		btTypedConstraint *		m_pConstraint;
		void *					m_pGameData;
		CPhysicsEnvironment *	m_pEnv;
		EConstraintType			m_type;
};

class CPhysicsSpring : public IPhysicsSpring, public CPhysicsConstraint {
	public:
		CPhysicsSpring(CPhysicsEnvironment *pEnv, CPhysicsObject *pReferenceObject, CPhysicsObject *pAttachedObject, btTypedConstraint *pConstraint);
		~CPhysicsSpring();

		void			GetEndpoints(Vector *worldPositionStart, Vector *worldPositionEnd);
		void			SetSpringConstant(float flSpringContant);
		void			SetSpringDamping(float flSpringDamping);
		void			SetSpringLength(float flSpringLength);

		// Get the starting object
		IPhysicsObject *GetStartObject();

		// Get the end object
		IPhysicsObject *GetEndObject();
};

// FIXME: I dont think we can implement this in Bullet anyways?
// We'll have to emulate this on bullet.
class CPhysicsConstraintGroup : public IPhysicsConstraintGroup
{
	public:
		CPhysicsConstraintGroup(const constraint_groupparams_t &params);
		~CPhysicsConstraintGroup(void);

		void	Activate();
		bool	IsInErrorState();
		void	ClearErrorState();
		void	GetErrorParams(constraint_groupparams_t *pParams);
		void	SetErrorParams(const constraint_groupparams_t &params);
		void	SolvePenetration(IPhysicsObject *pObj0, IPhysicsObject *pObj1);

	public:
		// Unexposed functions
		void	AddConstraint(CPhysicsConstraint *pConstraint);
		void	RemoveConstraint(CPhysicsConstraint *pConstraint);

	private:
		CUtlVector<CPhysicsConstraint *>	m_constraints;
};

// CONSTRAINT CREATION FUNCTIONS
CPhysicsSpring *CreateSpringConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, springparams_t *spring);
CPhysicsConstraint *CreateRagdollConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll);
CPhysicsConstraint *CreateHingeConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge);
CPhysicsConstraint *CreateFixedConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed);
CPhysicsConstraint *CreateSlidingConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding);
CPhysicsConstraint *CreateBallsocketConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket);
CPhysicsConstraint *CreatePulleyConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley);
CPhysicsConstraint *CreateLengthConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length);

#endif