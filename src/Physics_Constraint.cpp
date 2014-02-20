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
// Axis will be assumed as the forward vector (and assumed normalized)
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
		// The 2 points the pulley should run through in world space (such as being attached to a roof)
		btVector3	m_attachPointWS1;
		btVector3	m_attachPointWS2;

		btVector3	m_attachPointLS1;
		btVector3	m_attachPointLS2;

	public:
		btPulleyConstraint(btRigidBody &rbA, btRigidBody &rbB, const btVector3 &pivotInA, const btVector3 &pivotInB, const btVector3 &attachWS1, const btVector3 &attachWS2):
		btTypedConstraint(POINT2POINT_CONSTRAINT_TYPE, rbA, rbB) {

		}
};

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
			// Positions relative to objects
			btVector3 relA = m_rbA.getCenterOfMassTransform().getBasis() * getPivotInA();
			btVector3 relB = m_rbB.getCenterOfMassTransform().getBasis() * getPivotInB();

			// Exact world positions
			btVector3 posA = m_rbA.getCenterOfMassTransform().getOrigin() + relA;
			btVector3 posB = m_rbB.getCenterOfMassTransform().getOrigin() + relB;

			// Delta
			btVector3 del = posB - posA;
			btScalar currDist = del.length();

			info->m_numConstraintRows = 0;
			info->nub = 6; // FIXME: What does this even do?

			// Only need to solve the constraint if there is any error!
			if (currDist < m_mindist || currDist > m_maxdist) {
				info->m_numConstraintRows++;
				info->nub--;
			}
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
			btScalar currDist = del.length();

			btVector3 ortho = del / currDist; // Axis to solve along (normalized delta)
			// Linear axis for ref object(?)
			info->m_J1linearAxis[0] = ortho[0];
			info->m_J1linearAxis[1] = ortho[1];
			info->m_J1linearAxis[2] = ortho[2];
			
			// Linear axis for att object(?)
			if (info->m_J2linearAxis) {
				info->m_J2linearAxis[0] = -ortho[0];
				info->m_J2linearAxis[1] = -ortho[1];
				info->m_J2linearAxis[2] = -ortho[2];
			}

			// Angular axis (relative pos cross normal)
			btVector3 p, q;
			p = relA.cross(ortho);
			q = relB.cross(ortho);
			info->m_J1angularAxis[0] = p[0];
			info->m_J1angularAxis[1] = p[1];
			info->m_J1angularAxis[2] = p[2];

			if (info->m_J2angularAxis) {
				info->m_J2angularAxis[0] = -q[0];
				info->m_J2angularAxis[1] = -q[1];
				info->m_J2angularAxis[2] = -q[2];
			}

			btScalar rhs = 0;

			info->m_lowerLimit[0] = 0;
			info->m_upperLimit[0] = 0;

			// Keep the distance between min and max dist
			if (currDist < m_mindist) {
				rhs = (currDist - m_mindist) * info->fps * info->erp;
				info->m_lowerLimit[0] = -SIMD_INFINITY;
			} else if (currDist > m_maxdist) {
				rhs = (currDist - m_maxdist) * info->fps * info->erp;
				info->m_upperLimit[0] = SIMD_INFINITY;
			}

			info->m_constraintError[0] = rhs; // Constraint error (target rel velocity)
			info->cfm[0] = btScalar(0.f);
		}
};

class btUserConstraint : public btTypedConstraint {
	public:
		btUserConstraint(btRigidBody &rbA, btRigidBody &rbB, IPhysicsUserConstraint *pConstraint): btTypedConstraint(CONSTRAINT_TYPE_USER, rbA, rbB) {
			m_pUserConstraint = pConstraint;
		}

		void getInfo1(btConstraintInfo1 *pInfo) {
			physconstraintinfo_t info;
			info.numConstraintRows = 0;
			info.nub = 0;

			CPhysicsObject *pObjA, *pObjB;
			pObjA = (CPhysicsObject *)m_rbA.getUserPointer();
			pObjB = (CPhysicsObject *)m_rbB.getUserPointer();
			m_pUserConstraint->GetConstraintInfo(pObjA, pObjB, info);

			pInfo->m_numConstraintRows = info.numConstraintRows;
			pInfo->nub = info.nub;
		}

		void getInfo2(btConstraintInfo2 *pInfo) {
			physconstraintsolveinfo_t *solveinfo = (physconstraintsolveinfo_t *)stackalloc(pInfo->m_numConstraintRows);
			for (int i = 0; i < pInfo->m_numConstraintRows; i++) {
				solveinfo[i].Defaults();
			}

			CPhysicsObject *pObjA, *pObjB;
			pObjA = (CPhysicsObject *)m_rbA.getUserPointer();
			pObjB = (CPhysicsObject *)m_rbB.getUserPointer();
			m_pUserConstraint->GetConstraintSolveInfo(pObjA, pObjB, solveinfo, pInfo->m_numConstraintRows, pInfo->fps, pInfo->erp);

			for (int i = 0; i < pInfo->m_numConstraintRows; i++) {
				ConvertDirectionToBull(solveinfo[i].J1linearAxis, *(btVector3 *)&pInfo->m_J1linearAxis[i * pInfo->rowskip]);
				ConvertDirectionToBull(solveinfo[i].J1angularAxis, *(btVector3 *)&pInfo->m_J1angularAxis[i * pInfo->rowskip]);

				if (pInfo->m_J2linearAxis)
					ConvertDirectionToBull(solveinfo[i].J2linearAxis, *(btVector3 *)&pInfo->m_J2linearAxis[i * pInfo->rowskip]);

				if (pInfo->m_J2angularAxis)
					ConvertDirectionToBull(solveinfo[i].J2angularAxis, *(btVector3 *)&pInfo->m_J2angularAxis[i * pInfo->rowskip]);

				pInfo->cfm[i * pInfo->rowskip] = solveinfo[i].cfm;
				pInfo->m_constraintError[i * pInfo->rowskip] = solveinfo[i].constraintError;

				pInfo->m_lowerLimit[i * pInfo->rowskip] = solveinfo[i].lowerLimit;
				pInfo->m_upperLimit[i * pInfo->rowskip] = solveinfo[i].upperLimit;
			}
		}

	private:
		IPhysicsUserConstraint *m_pUserConstraint;
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
	// Can't use a for loop here :(
	btVector3 angUpperLimit;
	btVector3 angLowerLimit;

	btRotationalLimitMotor *motor = pConstraint->getRotationalLimitMotor(0);
	constraint_axislimit_t limit = ragdoll.axes[0];
	angUpperLimit.m_floats[0] = DEG2RAD(limit.maxRotation);
	angLowerLimit.m_floats[0] = DEG2RAD(limit.minRotation);
	if (limit.torque != 0) {
		motor->m_enableMotor = true;
		motor->m_targetVelocity = DEG2RAD(limit.angularVelocity);
		motor->m_maxMotorForce = DEG2RAD(HL2BULL(limit.torque));
	}

	// FIXME: Correct?
	motor = pConstraint->getRotationalLimitMotor(2);
	limit = ragdoll.axes[2];
	angUpperLimit.m_floats[1] = DEG2RAD(limit.maxRotation);
	angLowerLimit.m_floats[1] = DEG2RAD(limit.minRotation);
	if (limit.torque != 0) {
		motor->m_enableMotor = true;
		motor->m_targetVelocity = DEG2RAD(limit.angularVelocity);
		motor->m_maxMotorForce = DEG2RAD(HL2BULL(limit.torque));
	}

	motor = pConstraint->getRotationalLimitMotor(1);
	limit = ragdoll.axes[1];
	angUpperLimit.m_floats[2] = DEG2RAD(-limit.minRotation);
	angLowerLimit.m_floats[2] = DEG2RAD(-limit.maxRotation);
	if (limit.torque != 0) {
		motor->m_enableMotor = true;
		motor->m_targetVelocity = DEG2RAD(-limit.angularVelocity);
		motor->m_maxMotorForce = DEG2RAD(HL2BULL(-limit.torque));
	}

	pConstraint->setEnabled(ragdoll.isActive);

	pConstraint->setAngularUpperLimit(angUpperLimit);
	pConstraint->setAngularLowerLimit(angLowerLimit);

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
		pHinge->enableAngularMotor(true, DEG2RAD(hinge.hingeAxis.angularVelocity), DEG2RAD(HL2BULL(hinge.hingeAxis.torque)));

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

CPhysicsConstraint *CreateGearConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_gearparams_t &gear) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;

	btVector3 axes[2];
	for (int i = 0; i < 2; i++) {
		ConvertDirectionToBull(gear.objectLocalAxes[i], axes[i]);
	}

	btGearConstraint *pConstraint = new btGearConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), axes[0], axes[1], gear.ratio);
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pConstraint, CONSTRAINT_GEAR);
}

CPhysicsConstraintGroup *CreateConstraintGroup(CPhysicsEnvironment *pEnv, const constraint_groupparams_t &params) {
	if (!pEnv) return NULL;

	return new CPhysicsConstraintGroup(pEnv, params);
}

CPhysicsConstraint *CreateUserConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, IPhysicsUserConstraint *pUserConstraint) {
	Assert(pUserConstraint);

	NOT_IMPLEMENTED
	return NULL;
}