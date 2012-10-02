#include "StdAfx.h"

#include "CPhysicsFrictionSnapshot.h"
#include "CPhysicsObject.h"
#include "CPhysicsEnvironment.h"

#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

CPhysicsFrictionSnapshot::CPhysicsFrictionSnapshot(CPhysicsObject *pObject) {
	m_pObject = pObject;
	m_iCurContactPoint = 0;
	m_iCurManifold = 0;

	CPhysicsEnvironment *pEnv = pObject->GetVPhysicsEnvironment();
	btRigidBody *pBody = pObject->GetObject();
	int numManifolds = pEnv->GetBulletEnvironment()->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold *pManifold = pEnv->GetBulletEnvironment()->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject *pObjA = pManifold->getBody0();
		const btCollisionObject *pObjB = pManifold->getBody1();

		if (pManifold->getNumContacts() <= 0)
			continue;

		if (pObjA == pBody || pObjB == pBody) {
			m_manifolds.AddToTail(pManifold);
		}
	}
}

CPhysicsFrictionSnapshot::~CPhysicsFrictionSnapshot() {
	m_manifolds.RemoveAll();
}

bool CPhysicsFrictionSnapshot::IsValid() {
	return m_iCurManifold < m_manifolds.Count();
}

IPhysicsObject *CPhysicsFrictionSnapshot::GetObject(int index) {
	const btCollisionObject *pObjA = m_manifolds[m_iCurManifold]->getBody0();
	const btCollisionObject *pObjB = m_manifolds[m_iCurManifold]->getBody1();

	// TODO: Index 1 or 0?
	if (index == 1)
		return (IPhysicsObject *)pObjA->getUserPointer();
	else
		return (IPhysicsObject *)pObjB->getUserPointer();

	return NULL;
}

int CPhysicsFrictionSnapshot::GetMaterial(int index) {
	const btCollisionObject *pObjA = m_manifolds[m_iCurManifold]->getBody0();
	const btCollisionObject *pObjB = m_manifolds[m_iCurManifold]->getBody1();

	// TODO: Index 1 or 0?
	if (index == 1)
		return ((CPhysicsObject *)pObjA->getUserPointer())->GetMaterialIndex();
	else
		return ((CPhysicsObject *)pObjB->getUserPointer())->GetMaterialIndex();

	return 0;
}

void CPhysicsFrictionSnapshot::GetContactPoint(Vector &out) {
	btManifoldPoint bullManifoldPoint = m_manifolds[m_iCurManifold]->getContactPoint(m_iCurContactPoint);
	btVector3 bullPos = bullManifoldPoint.getPositionWorldOnA(); // TODO: A or B?
	ConvertPosToHL(bullPos, out);
}

void CPhysicsFrictionSnapshot::GetSurfaceNormal(Vector &out) {
	btManifoldPoint bullManifoldPoint = m_manifolds[m_iCurManifold]->getContactPoint(m_iCurContactPoint);
	ConvertPosToHL(bullManifoldPoint.m_normalWorldOnB, out);
}

float CPhysicsFrictionSnapshot::GetNormalForce() {
	btManifoldPoint bullManifoldPoint = m_manifolds[m_iCurManifold]->getContactPoint(m_iCurContactPoint);
	return bullManifoldPoint.m_appliedImpulse;
}

float CPhysicsFrictionSnapshot::GetEnergyAbsorbed() {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsFrictionSnapshot::RecomputeFriction() {
	NOT_IMPLEMENTED
}

void CPhysicsFrictionSnapshot::ClearFrictionForce() {
	NOT_IMPLEMENTED
}

void CPhysicsFrictionSnapshot::MarkContactForDelete() {
	NOT_IMPLEMENTED
}

void CPhysicsFrictionSnapshot::DeleteAllMarkedContacts(bool wakeObjects) {
	NOT_IMPLEMENTED
}

void CPhysicsFrictionSnapshot::NextFrictionData() {
	m_iCurContactPoint++;
	if (m_iCurContactPoint >= m_manifolds[m_iCurManifold]->getNumContacts()) {
		m_iCurManifold++;
		m_iCurContactPoint = 0;
	}
}

float CPhysicsFrictionSnapshot::GetFrictionCoefficient() {
	btManifoldPoint bullManifoldPoint = m_manifolds[m_iCurManifold]->getContactPoint(m_iCurContactPoint);
	NOT_IMPLEMENTED
	return 0;
}

