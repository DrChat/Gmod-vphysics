#include "StdAfx.h"

#include "CPhysicsObject.h"
#include "CPhysicsConstraint.h"
#include "CPhysicsEnvironment.h"

#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/***********************
* BULLET CONSTRAINTS
***********************/
// TODO: Somebody fill this out!
class btPulleyConstraint: public btTypedConstraint {
	protected:
	public:
};

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

// TODO: Find out how to use m_mindist to make ropes non-rigid, otherwise this is essentially a ballsocket.
class btDistanceConstraint : public btPoint2PointConstraint {
	protected:
		btScalar	m_dist;
	public:
		btDistanceConstraint(btRigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB, btScalar dist)
			: btPoint2PointConstraint(rbA, rbB, pivotInA, pivotInB)
		{
			m_dist = dist;
		}

		void getInfo1 (btConstraintInfo1* info) {
			info->m_numConstraintRows = 1;
			info->nub = 5;
		}

		void getInfo2 (btConstraintInfo2* info) {
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
CPhysicsConstraint::CPhysicsConstraint(CPhysicsEnvironment *pEnv, CPhysicsObject *pObject1, CPhysicsObject *pObject2, btTypedConstraint *pConstraint, EConstraintType type) {
	m_pObject1 = pObject1;
	m_pObject2 = pObject2;
	m_pConstraint = pConstraint;
	m_pEnv = pEnv;
	m_type = type;

	m_pEnv->GetBulletEnvironment()->addConstraint(m_pConstraint);
}

CPhysicsConstraint::~CPhysicsConstraint() {
	m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);
	delete m_pConstraint;
}

void CPhysicsConstraint::Activate(void) {
	m_pConstraint->setEnabled(true);
}

void CPhysicsConstraint::Deactivate(void) {
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
	NOT_IMPLEMENTED
}

/************************
* CREATION FUNCTIONS
************************/
CPhysicsConstraint *CreateRagdollConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	btTransform obj1Pos, obj2Pos;
	ConvertMatrixToBull(ragdoll.constraintToAttached, obj1Pos);
	ConvertMatrixToBull(ragdoll.constraintToReference, obj2Pos);
	CPhysicsObject *pObjA = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjB = (CPhysicsObject *)pAttachedObject;

	btPoint2PointConstraint *pBallsock = new btPoint2PointConstraint(*pObjA->GetObject(), *pObjB->GetObject(), obj1Pos.getOrigin(), obj2Pos.getOrigin());
	return new CPhysicsConstraint(pEnv, pObjA, pObjB, pBallsock, CONSTRAINT_RAGDOLL);
}

CPhysicsConstraint *CreateHingeConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	CPhysicsObject *pObjA = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjB = (CPhysicsObject *)pAttachedObject;

	btVector3 bullWorldPosition, bullWorldAxis;
	ConvertPosToBull(hinge.worldPosition, bullWorldPosition);
	ConvertDirectionToBull(hinge.worldAxisDirection, bullWorldAxis);

	//btHingeConstraint *pHinge = new btHingeConstraint();

	NOT_IMPLEMENTED;
	return NULL;
}

CPhysicsConstraint *CreateFixedConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	CPhysicsObject *pObjA = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjB = (CPhysicsObject *)pAttachedObject;
	btGeneric6DofConstraint *pWeld = new btGeneric6DofConstraint(*pObjA->GetObject(), *pObjB->GetObject(),
																pObjA->GetObject()->getWorldTransform().inverse() * pObjB->GetObject()->getWorldTransform(),
																btTransform::getIdentity(), true);
	pWeld->setLinearLowerLimit(btVector3(0,0,0));
	pWeld->setLinearUpperLimit(btVector3(0,0,0));
	pWeld->setAngularLowerLimit(btVector3(0,0,0));
	pWeld->setAngularUpperLimit(btVector3(0,0,0));

	return new CPhysicsConstraint(pEnv, pObjA, pObjB, pWeld, CONSTRAINT_FIXED);
}

CPhysicsConstraint *CreateSlidingConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	CPhysicsObject *pObjA = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjB = (CPhysicsObject *)pAttachedObject;

	// Position of attached object in reference object's local space.
	btTransform bullAttRefXform = btTransform::getIdentity();
	ConvertMatrixToBull(sliding.attachedRefXform, bullAttRefXform);
	
	btVector3 bullSlideAxisRef;
	ConvertDirectionToBull(sliding.slideAxisRef, bullSlideAxisRef);

	btTransform bullFrameInA = btTransform::getIdentity();
	bullFrameInA = ((btMassCenterMotionState *)pObjA->GetObject()->getMotionState())->m_centerOfMassOffset;
	bullFrameInA.setRotation(bullAttRefXform.getRotation());

	btTransform bullFrameInB = btTransform::getIdentity();

	// We need to know our own position and the other object's position in local space.
	// Figure out what makes the objects snap together when the constraint is created.
	// NOTES: The sliding constraint will line up with object B (spawning 2 washing machines near eachother and constraining them)
	// TODO: Figure out how to properly attach this to the world.

	btSliderConstraint *pSlider = new btSliderConstraint(*pObjA->GetObject(), *pObjB->GetObject(), bullFrameInA, bullFrameInB, true);

	// If linear min == lin max then there is no limit!
	if (sliding.limitMin != sliding.limitMax) {
		pSlider->setLowerLinLimit(ConvertDistanceToBull(sliding.limitMin));
		pSlider->setUpperLinLimit(ConvertDistanceToBull(sliding.limitMax));
	}

	pSlider->setLowerAngLimit(0);
	pSlider->setUpperAngLimit(0);

	return new CPhysicsConstraint(pEnv, pObjA, pObjB, pSlider, CONSTRAINT_SLIDING);
}

CPhysicsConstraint *CreateBallsocketConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {
	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(ballsocket.constraintPosition[0], obj1Pos);
	ConvertPosToBull(ballsocket.constraintPosition[1], obj2Pos);

	CPhysicsObject *pObjA = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjB = (CPhysicsObject *)pAttachedObject;
	PhysicsShapeInfo *shapeInfo1 = (PhysicsShapeInfo*)pObjA->GetObject()->getCollisionShape()->getUserPointer();
	PhysicsShapeInfo *shapeInfo2 = (PhysicsShapeInfo*)pObjB->GetObject()->getCollisionShape()->getUserPointer();

	if (shapeInfo1)
		obj1Pos -= shapeInfo1->massCenter;
	if (shapeInfo2)
		obj2Pos -= shapeInfo2->massCenter;

	btPoint2PointConstraint *pBallsock = new btPoint2PointConstraint(*pObjA->GetObject(), *pObjB->GetObject(), obj1Pos, obj2Pos);
	return new CPhysicsConstraint(pEnv, pObjA, pObjB, pBallsock, CONSTRAINT_BALLSOCKET);
}

CPhysicsConstraint *CreatePulleyConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	NOT_IMPLEMENTED;
	return NULL;
}

CPhysicsConstraint *CreateLengthConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(length.objectPosition[0], obj1Pos);
	ConvertPosToBull(length.objectPosition[1], obj2Pos);

	CPhysicsObject *pObjA = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjB = (CPhysicsObject *)pAttachedObject;
	PhysicsShapeInfo *shapeInfo1 = (PhysicsShapeInfo*)pObjA->GetObject()->getCollisionShape()->getUserPointer();
	PhysicsShapeInfo *shapeInfo2 = (PhysicsShapeInfo*)pObjB->GetObject()->getCollisionShape()->getUserPointer();

	if (shapeInfo1)
		obj1Pos -= shapeInfo1->massCenter;
	if (shapeInfo2)
		obj2Pos -= shapeInfo2->massCenter;

	btTransform obj1Trans, obj2Trans;
	obj1Trans.setIdentity();
	obj2Trans.setIdentity();

	obj1Trans.setOrigin(obj1Pos);
	obj2Trans.setOrigin(obj2Pos);

	btGeneric6DofConstraint *pLength = new btGeneric6DofConstraint(*pObjA->GetObject(), *pObjB->GetObject(), obj1Trans, obj2Trans, true);
	
	btScalar bullTotalLength = ConvertDistanceToBull(length.totalLength);
	pLength->setLinearUpperLimit(btVector3(bullTotalLength, bullTotalLength, bullTotalLength));

	if (length.minLength) {
		btScalar bullMinLength = ConvertDistanceToBull(length.minLength);
		pLength->setLinearLowerLimit(btVector3(bullMinLength, bullMinLength, bullMinLength));
	}

	//btPoint2PointConstraint *pLength = new btDistanceConstraint(*pObjA->GetObject(), *pObjB->GetObject(), obj1Pos, obj2Pos, HL2BULL(length.totalLength));
	return new CPhysicsConstraint(pEnv, pObjA, pObjB, pLength, CONSTRAINT_LENGTH);
}