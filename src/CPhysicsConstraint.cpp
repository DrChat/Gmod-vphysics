#include "StdAfx.h"

#include "CPhysicsObject.h"
#include "CPhysicsConstraint.h"
#include "CPhysicsEnvironment.h"

#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/*********************************
* CLASS CPhysicsConstraint
*********************************/
CPhysicsConstraint::CPhysicsConstraint(CPhysicsEnvironment *pEnv, CPhysicsObject* pObject1, CPhysicsObject* pObject2, btTypedConstraint *pConstraint)
{
	m_pObject1 = pObject1;
	m_pObject2 = pObject2;
	m_pConstraint = pConstraint;
	m_pEnv = pEnv;

	m_pEnv->GetBulletEnvironment()->addConstraint(m_pConstraint);
}

CPhysicsConstraint::~CPhysicsConstraint()
{
	m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);
	delete m_pConstraint;
}

void CPhysicsConstraint::Activate(void)
{
	m_pConstraint->setEnabled(true);
}

void CPhysicsConstraint::Deactivate(void)
{
	m_pConstraint->setEnabled(false);
}

/************************
* CREATION FUNCTIONS
************************/
CPhysicsConstraint *CreateRagdollConstraint(CPhysicsEnvironment *pEnv, CPhysicsObject *pReferenceObject, CPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	btTransform obj1Pos, obj2Pos;
	ConvertMatrixToBull(ragdoll.constraintToAttached, obj1Pos);
	ConvertMatrixToBull(ragdoll.constraintToReference, obj2Pos);
	CPhysicsObject *obj1 = (CPhysicsObject*)pReferenceObject, *obj2 = (CPhysicsObject*)pAttachedObject;

	btPoint2PointConstraint *ballsock = new btPoint2PointConstraint(*obj1->GetObject(), *obj2->GetObject(), obj1Pos.getOrigin(), obj2Pos.getOrigin());
	return new CPhysicsConstraint(pEnv, obj1, obj2, ballsock);
}

CPhysicsConstraint *CreateHingeConstraint(CPhysicsEnvironment *pEnv, CPhysicsObject *pReferenceObject, CPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	CPhysicsObject *pObjA = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjB = (CPhysicsObject *)pAttachedObject;

	//btHingeConstraint *hinge = new btHingeConstraint();

	NOT_IMPLEMENTED;
	return NULL;
}

CPhysicsConstraint *CreateFixedConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	CPhysicsObject *obj1 = (CPhysicsObject*)pReferenceObject, *obj2 = (CPhysicsObject*)pAttachedObject;
	btGeneric6DofConstraint *weld = new btGeneric6DofConstraint(*obj1->GetObject(), *obj2->GetObject(),
																obj1->GetObject()->getWorldTransform().inverse() * obj2->GetObject()->getWorldTransform(),
																btTransform::getIdentity(), true);
	weld->setLinearLowerLimit(btVector3(0,0,0));
	weld->setLinearUpperLimit(btVector3(0,0,0));
	weld->setAngularLowerLimit(btVector3(0,0,0));
	weld->setAngularUpperLimit(btVector3(0,0,0));

	return new CPhysicsConstraint(pEnv, obj1, obj2, weld);
}

CPhysicsConstraint *CreateSlidingConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	NOT_IMPLEMENTED;
	return NULL;
}

CPhysicsConstraint *CreateBallsocketConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {
	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(ballsocket.constraintPosition[0], obj1Pos);
	ConvertPosToBull(ballsocket.constraintPosition[1], obj2Pos);

	CPhysicsObject *obj1 = (CPhysicsObject*)pReferenceObject, *obj2 = (CPhysicsObject*)pAttachedObject;
	PhysicsShapeInfo *shapeInfo1 = (PhysicsShapeInfo*)obj1->GetObject()->getCollisionShape()->getUserPointer();
	PhysicsShapeInfo *shapeInfo2 = (PhysicsShapeInfo*)obj2->GetObject()->getCollisionShape()->getUserPointer();

	if (shapeInfo1)
		obj1Pos -= shapeInfo1->massCenter;
	if (shapeInfo2)
		obj2Pos -= shapeInfo2->massCenter;

	btPoint2PointConstraint *ballsock = new btPoint2PointConstraint(*obj1->GetObject(), *obj2->GetObject(), obj1Pos, obj2Pos);
	return new CPhysicsConstraint(pEnv, obj1, obj2, ballsock);
}

CPhysicsConstraint *CreatePulleyConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	NOT_IMPLEMENTED;
	return NULL;
}

CPhysicsConstraint *CreateLengthConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(length.objectPosition[0], obj1Pos);
	ConvertPosToBull(length.objectPosition[1], obj2Pos);

	CPhysicsObject *obj1 = (CPhysicsObject*)pReferenceObject, *obj2 = (CPhysicsObject*)pAttachedObject;
	PhysicsShapeInfo *shapeInfo1 = (PhysicsShapeInfo*)obj1->GetObject()->getCollisionShape()->getUserPointer();
	PhysicsShapeInfo *shapeInfo2 = (PhysicsShapeInfo*)obj2->GetObject()->getCollisionShape()->getUserPointer();

	if (shapeInfo1)
		obj1Pos -= shapeInfo1->massCenter;
	if (shapeInfo2)
		obj2Pos -= shapeInfo2->massCenter;

	btPoint2PointConstraint *constraint = new btDistanceConstraint(*obj1->GetObject(), *obj2->GetObject(), obj1Pos, obj2Pos, HL2BULL(length.minLength), HL2BULL(length.totalLength));
	return new CPhysicsConstraint(pEnv, obj1, obj2, constraint);
}