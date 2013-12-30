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

// Convert an axis to a matrix (where angle around axis does not matter)
// Axis will be assumed as the forward vector
void bullAxisToMatrix(const btVector3 &axis, btMatrix3x3 &matrix) {
	btVector3 wup(0, 1, 0);

	// Handle cases where the axis is really close to up/down vector
	// Dot = cos(theta) (for dummies), looking for 0(1) to 180(-1) degrees difference
	btScalar dot = wup.dot(axis);
	if ((dot > 1.0f - SIMD_EPSILON) || (dot < -1.0f + SIMD_EPSILON)) {
		// This part may be broken!
		// Axis is really close to/is the up/down vector! We'll have to use a side vector as a base instead.
		btVector3 wside(0, 0, 1);

		btVector3 up = wside.cross(axis);
		btVector3 side = up.cross(axis);

		// Normalize the cross products, as they may not be unit length.
		side.normalize();
		up.normalize();

		matrix.setValue(axis.x(), up.x(), side.x(),
						axis.y(), up.y(), side.y(),
						axis.z(), up.z(), side.z());
	} else {
		btVector3 side = wup.cross(axis);
		btVector3 up = side.cross(axis);

		// Cross products may not be unit length!
		side.normalize();
		up.normalize();

		matrix.setValue(axis.x(), up.x(), side.x(),
						axis.y(), up.y(), side.y(),
						axis.z(), up.z(), side.z());
	}
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

		btVector3	m_attachPointLS1;
		btVector3	m_attachPointLS2;

	public:
		btPulleyConstraint(btRigidBody &rbA, btRigidBody &rbB, const btVector3 &pivotInA, const btVector3 &pivotInB, const btVector3 &attachWS1, const btVector3 &attachWS2):
		btTypedConstraint(POINT2POINT_CONSTRAINT_TYPE, rbA, rbB) {

		}
};

// TODO: Finish this (for ropes)
class btLengthConstraint: public btPoint2PointConstraint {
	protected:
		btScalar	m_mindist;
		btScalar	m_maxdist;
	public:
		btLengthConstraint(btRigidBody &rbA, btRigidBody &rbB, const btVector3 &pivotInA, const btVector3 &pivotInB, btScalar minDist, btScalar maxDist):
		btPoint2PointConstraint(rbA, rbB, pivotInA, pivotInB) {
			m_mindist = minDist;
			m_maxdist = maxDist;
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

			btScalar rhs = 0;

			// Keep the distance between min and max dist
			if (currDist < m_mindist) {
				rhs = (currDist - m_mindist) * info->fps * info->erp;
			} /*else if (currDist > m_maxdist) {
				rhs = (currDist - m_maxdist) * info->fps * info->erp;
			}
			*/

			info->m_constraintError[0] = rhs;
			info->cfm[0] = btScalar(0.f);
			info->m_lowerLimit[0] = -SIMD_INFINITY;
			info->m_upperLimit[0] = SIMD_INFINITY;
		}
};

class btDistanceConstraint : public btPoint2PointConstraint {
	protected:
		btScalar	m_dist;
	public:
		btDistanceConstraint(btRigidBody& rbA, btRigidBody& rbB, const btVector3& pivotInA, const btVector3& pivotInB, btScalar dist):
		btPoint2PointConstraint(rbA, rbB, pivotInA, pivotInB) {
			m_dist = dist;
		}

		void getInfo1(btConstraintInfo1 *info) {
			info->m_numConstraintRows = 1;
			info->nub = 5;
		}

		void getInfo2(btConstraintInfo2 *info) {
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
	m_bRemovedFromEnv = false;

	if (m_type == CONSTRAINT_RAGDOLL) {
		m_pEnv->GetBulletEnvironment()->addConstraint(m_pConstraint, true);
	} else {
		m_pEnv->GetBulletEnvironment()->addConstraint(m_pConstraint);
	}

	m_pConstraint->setUserConstraintPtr(this);

	if (pReferenceObject)
		pReferenceObject->AttachedToConstraint(this);

	if (pAttachedObject)
		pAttachedObject->AttachedToConstraint(this);

	if (m_pGroup) {
		m_pGroup->AddConstraint(this);
	}
}

CPhysicsConstraint::~CPhysicsConstraint() {
	if (!m_bRemovedFromEnv)
		m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);

	if (m_pReferenceObject)
		m_pReferenceObject->DetachedFromConstraint(this);

	if (m_pAttachedObject)
		m_pAttachedObject->DetachedFromConstraint(this);

	if (m_pGroup) {
		m_pGroup->RemoveConstraint(this);
	}

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
	if (m_type == CONSTRAINT_HINGE) {
		btHingeConstraint *pHinge = (btHingeConstraint *)m_pConstraint;

		// FIXME: Probably not the right conversions!
		pHinge->enableAngularMotor(true, DEG2RAD(rotSpeed), DEG2RAD(maxAngularImpulse));
	}

	NOT_IMPLEMENTED
	//m_pConstraint->enableAngularMotor();
}

void CPhysicsConstraint::UpdateRagdollTransforms(const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached) {
	if (m_type != CONSTRAINT_RAGDOLL) return;
	NOT_IMPLEMENTED
}

bool CPhysicsConstraint::GetConstraintTransform(matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached) const {
	if (!pConstraintToReference && !pConstraintToAttached) return false;

	NOT_IMPLEMENTED
	return false;
}

bool CPhysicsConstraint::GetConstraintParams(constraint_breakableparams_t *pParams) const {
	if (!pParams) return false;

	NOT_IMPLEMENTED
	return false;
}

void CPhysicsConstraint::OutputDebugInfo() {
	Msg("-------------------\n");
	Msg("%s constraint\n", GetConstraintName(m_type));
}

// UNEXPOSED
// This function is called before the rigid body is deleted.
void CPhysicsConstraint::ObjectDestroyed(CPhysicsObject *pObject) {
	if (pObject != m_pAttachedObject && pObject != m_pReferenceObject) {
		AssertMsg(0, "ObjectDestroyed called with object that isn't part of this constraint!");
		return;
	}

	if (pObject == m_pAttachedObject)
		m_pAttachedObject = NULL;

	if (pObject == m_pReferenceObject)
		m_pReferenceObject = NULL;

	// Constraint is no longer valid due to one of its objects being removed, so stop simulating it.
	if (!m_bRemovedFromEnv) {
		m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);
		m_bRemovedFromEnv = true;
	}

	// Tell the game that this constraint was broken.
	m_pEnv->HandleConstraintBroken(this);
}

// UNEXPOSED
EConstraintType CPhysicsConstraint::GetType() {
	return m_type;
}

// UNEXPOSED
btTypedConstraint *CPhysicsConstraint::GetConstraint() {
	return m_pConstraint;
}

/*********************************
* CLASS CPhysicsConstraintGroup
*********************************/

CPhysicsConstraintGroup::CPhysicsConstraintGroup(CPhysicsEnvironment *pEnv, const constraint_groupparams_t &params) {
	m_errorParams = params;
	m_pEnvironment = pEnv;
}

CPhysicsConstraintGroup::~CPhysicsConstraintGroup() {

}

void CPhysicsConstraintGroup::Activate() {
	for (int i = 0; i < m_constraints.Count(); i++) {
		m_constraints[i]->Activate();
	}
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

CPhysicsSpring::CPhysicsSpring(CPhysicsEnvironment *pEnv, CPhysicsObject *pReferenceObject, CPhysicsObject *pAttachedObject, btTypedConstraint *pConstraint) {
	m_pEnvironment = pEnv;

	m_pReferenceObject = pReferenceObject;
	m_pAttachedObject = pAttachedObject;
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

// NOT COMPLETE
CPhysicsConstraint *CreateRagdollConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btRigidBody *objRef = pObjRef->GetObject();
	btRigidBody *objAtt = pObjAtt->GetObject();

	btTransform constraintToReference, constraintToAttached;
	ConvertMatrixToBull(ragdoll.constraintToReference, constraintToReference); // constraintToReference is ALWAYS the identity matrix.
	ConvertMatrixToBull(ragdoll.constraintToAttached, constraintToAttached);

	btTransform constraintWorldTrans = constraintToReference.inverse() * objRef->getWorldTransform();
	btTransform bullAFrame = objRef->getWorldTransform().inverse() * constraintWorldTrans;
	btTransform bullBFrame = objAtt->getWorldTransform().inverse() * constraintWorldTrans;

	btGeneric6DofConstraint *pConstraint = new btGeneric6DofConstraint(*objRef, *objAtt, bullAFrame, bullBFrame, true);

	// Build up our limits
	btVector3 angUpperLimit;
	btVector3 angLowerLimit;

	constraint_axislimit_t limit = ragdoll.axes[0];
	angUpperLimit.m_floats[0] = DEG2RAD(limit.maxRotation);
	angLowerLimit.m_floats[0] = DEG2RAD(limit.minRotation);

	// FIXME: Correct?
	limit = ragdoll.axes[2];
	angUpperLimit.m_floats[1] = DEG2RAD(limit.minRotation);
	angLowerLimit.m_floats[1] = DEG2RAD(limit.maxRotation);

	limit = ragdoll.axes[1];
	angUpperLimit.m_floats[2] = DEG2RAD(-limit.maxRotation);
	angLowerLimit.m_floats[2] = DEG2RAD(-limit.minRotation);

	pConstraint->setEnabled(ragdoll.isActive);

	// Set axis limits
	
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pConstraint, CONSTRAINT_RAGDOLL);
}

CPhysicsConstraint *CreateHingeConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btRigidBody *objRef = pObjRef->GetObject();
	btRigidBody *objAtt = pObjAtt->GetObject();

	btVector3 bullWorldPosition, bullWorldAxis;
	ConvertPosToBull(hinge.worldPosition, bullWorldPosition);
	ConvertDirectionToBull(hinge.worldAxisDirection, bullWorldAxis);

	btMatrix3x3 worldMatrix;
	bullAxisToMatrix(bullWorldAxis, worldMatrix);

	// Setup the constraint to be on the expected axis (flip fwd and right)
	btVector3 fwd, up, right;
	fwd		= worldMatrix.getColumn(0);
	up		= worldMatrix.getColumn(1);
	right	= worldMatrix.getColumn(2);

	worldMatrix.setValue(right.x(), up.x(), fwd.x(),
						 right.y(), up.y(), fwd.y(),
						 right.z(), up.z(), fwd.z());

	// Constraint world transform
	btTransform worldTrans(worldMatrix, bullWorldPosition);

	// Setup local transforms inside of the objects
	btTransform refTransform = objRef->getWorldTransform().inverse() * worldTrans;
	btTransform attTransform = objAtt->getWorldTransform().inverse() * worldTrans;

	btHingeConstraint *pHinge = new btHingeConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), refTransform, attTransform);

	// FIXME: Are we converting the torque correctly? Bullet takes a "max motor impulse"
	if (hinge.hingeAxis.torque)
		pHinge->enableAngularMotor(true, DEG2RAD(hinge.hingeAxis.angularVelocity), DEG2RAD(hinge.hingeAxis.torque));

	if (hinge.hingeAxis.minRotation != hinge.hingeAxis.maxRotation)
		pHinge->setLimit(DEG2RAD(hinge.hingeAxis.minRotation), DEG2RAD(hinge.hingeAxis.maxRotation));

	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pHinge, CONSTRAINT_HINGE);
}

CPhysicsConstraint *CreateFixedConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btFixedConstraint *pWeld = new btFixedConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(),
																pObjRef->GetObject()->getWorldTransform().inverse() * pObjAtt->GetObject()->getWorldTransform(),
																btTransform::getIdentity());

	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pWeld, CONSTRAINT_FIXED);
}

CPhysicsConstraint *CreateSlidingConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btRigidBody *objRef = pObjRef->GetObject();
	btRigidBody *objAtt = pObjAtt->GetObject();

	// Axis to slide on in reference object space
	btVector3 slideAxisRef;
	ConvertDirectionToBull(sliding.slideAxisRef, slideAxisRef);

	// Reference -> attached object transform
	btTransform refToAttXform = objAtt->getWorldTransform().inverse() * objRef->getWorldTransform();

	// Attached -> reference object transform
	btTransform attToRefXform = objRef->getWorldTransform().inverse() * objAtt->getWorldTransform();

	// Build reference matrix
	btMatrix3x3 refMatrix;
	bullAxisToMatrix(slideAxisRef, refMatrix);

	btQuaternion refQuat;
	refMatrix.getRotation(refQuat);

	// Important to be the same rotation around axis (prevent attached object from flipping out)
	btQuaternion attQuat = refToAttXform.getRotation() * refQuat;

	// Final frames
	btTransform refFrame = btTransform::getIdentity();
	refFrame.setBasis(refMatrix);

	// Attached frame will be where reference frame is (keep attached object in it's initial spot instead of snapping to the center)
	btTransform attFrame = attToRefXform.inverse();
	attFrame.setRotation(attQuat);

	btSliderConstraint *pSlider = new btSliderConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), refFrame, attFrame, true);

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

	btPoint2PointConstraint *pLength = new btLengthConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), obj1Pos, obj2Pos, ConvertDistanceToBull(length.minLength), ConvertDistanceToBull(length.totalLength));
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pLength, CONSTRAINT_LENGTH);
}

CPhysicsConstraintGroup *CreateConstraintGroup(CPhysicsEnvironment *pEnv, const constraint_groupparams_t &params) {
	if (!pEnv) return NULL;

	return new CPhysicsConstraintGroup(pEnv, params);
}