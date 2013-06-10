#include "StdAfx.h"

#include "Physics_Object.h"
#include "Physics_Constraint.h"
#include "Physics_Environment.h"

#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/**********************
* Misc. Functions
**********************/
// I hope these work!
inline void TransformWorldToLocal(const btTransform &trans, const btRigidBody &obj, btTransform &out) {
	out = trans * obj.getWorldTransform().inverse();
}

inline void TransformLocalToWorld(const btTransform &trans, const btRigidBody &obj, btTransform &out) {
	out = trans * obj.getWorldTransform();
}

inline void GraphicTransformWorldToLocal(const btTransform &trans, const btRigidBody &obj, btTransform &out) {
	out = (trans * ((btMassCenterMotionState *)obj.getMotionState())->m_centerOfMassOffset) * obj.getWorldTransform();
}

inline void GraphicTransformLocalToWorld(const btTransform &trans, const btRigidBody &obj, btTransform &out) {
	out = (trans  * ((btMassCenterMotionState *)obj.getMotionState())->m_centerOfMassOffset) * obj.getWorldTransform();
}

const char *GetConstraintName(EConstraintType type) {
	switch (type) {
		case CONSTRAINT_UNKNOWN:
			return "unknown";
		case CONSTRAINT_RAGDOLL:
			return "ragdoll";
		case CONSTRAINT_HINGE:
			return "hinge";
		case CONSTRAINT_FIXED:
			return "fixed";
		case CONSTRAINT_SLIDING:
			return "sliding";
		case CONSTRAINT_BALLSOCKET:
			return "ballsocket";
		case CONSTRAINT_PULLEY:
			return "pulley";
		case CONSTRAINT_LENGTH:
			return "length";
		case CONSTRAINT_SPRING:
			return "spring";
		default:
			return "invalid";
	}
}

/***********************
* BULLET CONSTRAINTS
***********************/
// TODO: Somebody fill this out!
class btPulleyConstraint: public btTypedConstraint {
	protected:
		// The 2 points the pulley should run through (such as being attached to a roof)
		btVector3	m_attachPointWS1;
		btVector3	m_attachPointWS2;
	public:
};

// TODO: Finish this (for ropes)
class btLengthConstraint: public btPoint2PointConstraint {
	protected:
		btScalar	m_mindist;
		btScalar	m_maxdist;
	public:
		btLengthConstraint(btRigidBody &rbA, btRigidBody &rbB, const btVector3 &pivotInA, const btVector3 &pivotInB, btScalar minDist, btScalar maxDist)
			: btPoint2PointConstraint(rbA, rbB, pivotInA, pivotInB)
		{
			m_mindist = minDist;
			m_maxdist = maxDist;
		}
};

class btDistanceConstraint : public btPoint2PointConstraint {
	protected:
		btScalar	m_dist;
	public:
		btDistanceConstraint(btRigidBody& rbA, btRigidBody& rbB, const btVector3& pivotInA, const btVector3& pivotInB, btScalar dist)
			: btPoint2PointConstraint(rbA, rbB, pivotInA, pivotInB)
		{
			m_dist = dist;
		}

		void getInfo1 (btConstraintInfo1 *info) {
			info->m_numConstraintRows = 1;
			info->nub = 5;
		}

		void getInfo2 (btConstraintInfo2 *info) {
			// Positions relative to objects
			btVector3 relA = m_rbA.getCenterOfMassTransform().getBasis() * getPivotInA();
			btVector3 relB = m_rbB.getCenterOfMassTransform().getBasis() * getPivotInB();

			// Exact world positions
			btVector3 posA = m_rbA.getCenterOfMassTransform().getOrigin() + relA;
			btVector3 posB = m_rbB.getCenterOfMassTransform().getOrigin() + relB;

			// Delta
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
			btScalar rhs = (currDist - m_dist) * info->fps * info->erp;
			info->m_constraintError[0] = rhs;
			info->cfm[0] = btScalar(0.f);
			info->m_lowerLimit[0] = -SIMD_INFINITY;
			info->m_upperLimit[0] = SIMD_INFINITY;
		}
};

/*********************************
* CLASS CPhysicsConstraint
*********************************/

CPhysicsConstraint::CPhysicsConstraint(CPhysicsEnvironment *pEnv, IPhysicsConstraintGroup *pGroup, CPhysicsObject *pReferenceObject, CPhysicsObject *pAttachedObject, btTypedConstraint *pConstraint, EConstraintType type) {
	m_pReferenceObject = pReferenceObject;
	m_pAttachedObject = pAttachedObject;
	m_pConstraint = pConstraint;
	m_pGroup = (CPhysicsConstraintGroup *)pGroup; // May be NULL in the case of spring constraints.
	m_pEnv = pEnv;
	m_type = type;

	if (m_type == CONSTRAINT_RAGDOLL) {
		m_pEnv->GetBulletEnvironment()->addConstraint(m_pConstraint, true);
	} else {
		m_pEnv->GetBulletEnvironment()->addConstraint(m_pConstraint);
	}

	if (pReferenceObject)
		pReferenceObject->AttachedToConstraint(this);

	if (pAttachedObject)
		pAttachedObject->AttachedToConstraint(this);

	if (m_pGroup) {
		m_pGroup->AddConstraint(this);
	}
}

CPhysicsConstraint::~CPhysicsConstraint() {
	m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);

	if (m_pReferenceObject)
		m_pReferenceObject->DetachedFromConstraint(this);

	if (m_pAttachedObject)
		m_pAttachedObject->DetachedFromConstraint(this);

	delete m_pConstraint;
}

void CPhysicsConstraint::Activate() {
	m_pConstraint->setEnabled(true);
}

void CPhysicsConstraint::Deactivate() {
	m_pConstraint->setEnabled(false);
}

void CPhysicsConstraint::SetLinearMotor(float speed, float maxLinearImpulse) {
	NOT_IMPLEMENTED
}

void CPhysicsConstraint::SetAngularMotor(float rotSpeed, float maxAngularImpulse) {
	NOT_IMPLEMENTED
	//m_pConstraint->enableAngularMotor();
}

void CPhysicsConstraint::UpdateRagdollTransforms(const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached) {
	if (m_type != CONSTRAINT_RAGDOLL) return;
	NOT_IMPLEMENTED
}

bool CPhysicsConstraint::GetConstraintTransform(matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached) const {
	NOT_IMPLEMENTED
	return false;
}

bool CPhysicsConstraint::GetConstraintParams(constraint_breakableparams_t *pParams) const {
	NOT_IMPLEMENTED
	return false;
}

void CPhysicsConstraint::OutputDebugInfo() {
	Msg("-------------------\n");
	Msg("%s constraint\n", GetConstraintName(m_type));
}

void CPhysicsConstraint::ObjectDestroyed(CPhysicsObject *pObject) {
	if (pObject == m_pAttachedObject)
		m_pAttachedObject = NULL;

	if (pObject == m_pReferenceObject)
		m_pReferenceObject = NULL;

	// HACK: We'll have to remove ourselves from the environment. Hopefully the game is about to destroy us anyways, otherwise prepare for explosions.
	m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);
}

EConstraintType CPhysicsConstraint::GetType() {
	return m_type;
}

/*********************************
* CLASS CPhysicsConstraintGroup
*********************************/

CPhysicsConstraintGroup::CPhysicsConstraintGroup(const constraint_groupparams_t &params) {
	m_errorParams = params;
}

CPhysicsConstraintGroup::~CPhysicsConstraintGroup() {

}

void CPhysicsConstraintGroup::Activate() {
	NOT_IMPLEMENTED
}

bool CPhysicsConstraintGroup::IsInErrorState() {
	// Called every frame
	// NOT_IMPLEMENTED
	return false;
}

void CPhysicsConstraintGroup::ClearErrorState() {
	// Called every frame
	// NOT_IMPLEMENTED
}

void CPhysicsConstraintGroup::GetErrorParams(constraint_groupparams_t *pParams) {
	if (pParams)
		*pParams = m_errorParams;
}

void CPhysicsConstraintGroup::SetErrorParams(const constraint_groupparams_t &params) {
	m_errorParams = params;
}

void CPhysicsConstraintGroup::SolvePenetration(IPhysicsObject *pObj0, IPhysicsObject *pObj1) {
	NOT_IMPLEMENTED
}

// UNEXPOSED
void CPhysicsConstraintGroup::AddConstraint(CPhysicsConstraint *pConstraint) {
	m_constraints.AddToTail(pConstraint);
}

// UNEXPOSED
void CPhysicsConstraintGroup::RemoveConstraint(CPhysicsConstraint *pConstraint) {
	m_constraints.FindAndRemove(pConstraint);
}

/************************
* CLASS CPhysicsSpring
************************/
// REMEMBER: If you allocate anything inside IPhysicsSpring, you'll have to REWRITE CPhysicsEnvironment::DestroySpring!!!

CPhysicsSpring::CPhysicsSpring(CPhysicsEnvironment *pEnv, CPhysicsObject *pReferenceObject, CPhysicsObject *pAttachedObject, btTypedConstraint *pConstraint)
	: CPhysicsConstraint(pEnv, NULL, pReferenceObject, pAttachedObject, pConstraint, CONSTRAINT_SPRING) {
	
}

CPhysicsSpring::~CPhysicsSpring() {

}

void CPhysicsSpring::GetEndpoints(Vector *worldPositionStart, Vector *worldPositionEnd) {
	if (!worldPositionStart && !worldPositionEnd) return;
	NOT_IMPLEMENTED
}

void CPhysicsSpring::SetSpringConstant(float flSpringContant) {
	NOT_IMPLEMENTED
}

void CPhysicsSpring::SetSpringDamping(float flSpringDamping) {
	NOT_IMPLEMENTED
}

void CPhysicsSpring::SetSpringLength(float flSpringLength) {
	NOT_IMPLEMENTED
}

IPhysicsObject *CPhysicsSpring::GetStartObject() {
	return m_pReferenceObject;
}

IPhysicsObject *CPhysicsSpring::GetEndObject() {
	return m_pAttachedObject;
}

/************************
* CREATION FUNCTIONS
************************/

// NOT COMPLETE
CPhysicsSpring *CreateSpringConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, springparams_t *spring) {
	if (!spring) return NULL;

	btVector3 bullRefPos, bullAttPos;
	ConvertPosToBull(spring->startPosition, bullRefPos);
	ConvertPosToBull(spring->endPosition, bullAttPos);

	NOT_IMPLEMENTED
	return NULL;
}

CPhysicsConstraint *CreateRagdollConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;

	btTransform constraintToReference, constraintToAttached;
	ConvertMatrixToBull(ragdoll.constraintToReference, constraintToReference); // constraintToReference is ALWAYS the identity matrix.
	ConvertMatrixToBull(ragdoll.constraintToAttached, constraintToAttached);

	btTransform bullAFrame = constraintToReference;
	btTransform bullBFrame = constraintToAttached;

	// We shouldn't be using a cone twist constraint! old vphysics uses a "limited ballsocket" constraint
	btConeTwistConstraint *pConstraint = new btConeTwistConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), bullAFrame, bullBFrame);

	pConstraint->setAngularOnly(ragdoll.onlyAngularLimits);

	// Set axis limits
	// swing span 1, swing span 2, twist limit
	// Bullet only supports setting the maximum limits, not minimums!
	
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pConstraint, CONSTRAINT_RAGDOLL);
}

// NOT COMPLETE
CPhysicsConstraint *CreateHingeConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btRigidBody *bullObjRef = pObjRef->GetObject();
	btRigidBody *bullObjAtt = pObjAtt->GetObject();

	btVector3 bullWorldPosition, bullWorldAxis;
	ConvertPosToBull(hinge.worldPosition, bullWorldPosition);
	ConvertDirectionToBull(hinge.worldAxisDirection, bullWorldAxis);

	// How do we transfer the axis into object local coords?
	btVector3 refAxis = bullWorldAxis * bullObjRef->getWorldTransform().getBasis();
	btVector3 attAxis = -bullWorldAxis * bullObjAtt->getWorldTransform().getBasis();

	// FIXME: Position is correct, but rotation isn't
	btVector3 bullPivotA = bullWorldPosition - bullObjRef->getWorldTransform().getOrigin();
	btVector3 bullPivotB = bullWorldPosition - bullObjAtt->getWorldTransform().getOrigin();

	btHingeConstraint *pHinge = new btHingeConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), bullPivotA, bullPivotB, refAxis, attAxis);
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pHinge, CONSTRAINT_HINGE);
}

CPhysicsConstraint *CreateFixedConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btGeneric6DofConstraint *pWeld = new btGeneric6DofConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(),
																pObjRef->GetObject()->getWorldTransform().inverse() * pObjAtt->GetObject()->getWorldTransform(),
																btTransform::getIdentity(), true);
	pWeld->setLinearLowerLimit(btVector3(0, 0, 0));
	pWeld->setLinearUpperLimit(btVector3(0, 0, 0));
	pWeld->setAngularLowerLimit(btVector3(0, 0, 0));
	pWeld->setAngularUpperLimit(btVector3(0, 0, 0));

	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pWeld, CONSTRAINT_FIXED);
}

// NOT COMPLETE
CPhysicsConstraint *CreateSlidingConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btRigidBody *objRef = pObjRef->GetObject();
	btRigidBody *objAtt = pObjAtt->GetObject();

	// Position of attached object space relative to reference object space.
	btTransform bullAttRefXform = btTransform::getIdentity();
	ConvertMatrixToBull(sliding.attachedRefXform, bullAttRefXform);
	
	// Axis to slide on in reference object space
	btVector3 bullSlideAxisRef;
	ConvertDirectionToBull(sliding.slideAxisRef, bullSlideAxisRef);

	// TODO: How do we convert the slide axis to a btQuaternion?

	// Sliders connect the centers of each object
	btTransform bullFrameInA = btTransform::getIdentity();
	bullFrameInA.setRotation(btQuaternion(btVector3(0, 1, 0), 1) * objRef->getWorldTransform().getRotation().inverse());

	btTransform bullFrameInB = btTransform::getIdentity();
	bullFrameInB.setRotation(btQuaternion(btVector3(0, 1, 0), 1) * objAtt->getWorldTransform().getRotation().inverse());

	btSliderConstraint *pSlider = new btSliderConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), bullFrameInA, bullFrameInB, true);

	// If linear min == lin max then there is no limit!
	if (sliding.limitMin != sliding.limitMax) {
		pSlider->setLowerLinLimit(ConvertDistanceToBull(sliding.limitMin));
		pSlider->setUpperLinLimit(ConvertDistanceToBull(sliding.limitMax));
	}

	pSlider->setLowerAngLimit(0);
	pSlider->setUpperAngLimit(0);

	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pSlider, CONSTRAINT_SLIDING);
}

CPhysicsConstraint *CreateBallsocketConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;

	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(ballsocket.constraintPosition[0], obj1Pos);
	ConvertPosToBull(ballsocket.constraintPosition[1], obj2Pos);

	obj1Pos -= ((btMassCenterMotionState *)pObjRef->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();
	obj2Pos -= ((btMassCenterMotionState *)pObjAtt->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();

	btPoint2PointConstraint *pBallsock = new btPoint2PointConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), obj1Pos, obj2Pos);
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pBallsock, CONSTRAINT_BALLSOCKET);
}

// NOT COMPLETE
CPhysicsConstraint *CreatePulleyConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	// TODO: Bullet has no default pulley constraint. Make one.
	NOT_IMPLEMENTED
	return NULL;
}

// NOT COMPLETE
CPhysicsConstraint *CreateLengthConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(length.objectPosition[0], obj1Pos);
	ConvertPosToBull(length.objectPosition[1], obj2Pos);

	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;

	obj1Pos -= ((btMassCenterMotionState *)pObjRef->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();
	obj2Pos -= ((btMassCenterMotionState *)pObjAtt->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();

	btPoint2PointConstraint *pLength = new btDistanceConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), obj1Pos, obj2Pos, ConvertDistanceToBull(length.totalLength));
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pLength, CONSTRAINT_LENGTH);
}