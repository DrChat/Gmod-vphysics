#include "StdAfx.h"

#include "CPhysicsFrictionSnapshot.h"
#include "CPhysicsObject.h"
#include "CPhysicsEnvironment.h"

CPhysicsFrictionSnapshot::CPhysicsFrictionSnapshot(CPhysicsObject* pObject) {
	// Didn't really get anywhere with this
	/*
	m_pObject = pObject;
	m_manifold = 0;
	m_contactpoint = 0;
	btRigidBody* body = pObject->GetObject();
	btDynamicsWorld* pEnv = pObject->GetVPhysicsEnvironment()->GetBulletEnvironment();
	btDispatcher* dispatch = pEnv->getDispatcher();
	int count = dispatch->getNumManifolds();
	for (int i = 0; i < count; i++) {
		btPersistentManifold* manifold = dispatch->getManifoldByIndexInternal(i);
		void* body0 = manifold->getBody0();
		void* body1 = manifold->getBody1();
		if (body0 == body || body1 == body) {
			m_manifolds.AddToTail(manifold);
		}
	}
	*/
}

CPhysicsFrictionSnapshot::~CPhysicsFrictionSnapshot() {

}

bool CPhysicsFrictionSnapshot::IsValid() {
	//return m_manifold < m_manifolds.Count();
	NOT_IMPLEMENTED;
	return false;
}

IPhysicsObject* CPhysicsFrictionSnapshot::GetObject(int index) {
	NOT_IMPLEMENTED;
	return NULL;
}

int CPhysicsFrictionSnapshot::GetMaterial(int index) {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsFrictionSnapshot::GetContactPoint(Vector& out) {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::GetSurfaceNormal(Vector& out) {
	NOT_IMPLEMENTED;
}

float CPhysicsFrictionSnapshot::GetNormalForce() {
	NOT_IMPLEMENTED;
	return 0;
}

float CPhysicsFrictionSnapshot::GetEnergyAbsorbed() {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsFrictionSnapshot::RecomputeFriction() {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::ClearFrictionForce() {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::MarkContactForDelete() {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::DeleteAllMarkedContacts(bool wakeObjects) {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::NextFrictionData() {
	/*
	m_contactpoint++;
	if (m_contactpoint >= m_manifolds[m_manifold]->getNumContacts()) {
		m_manifold++;
		m_contactpoint = 0;
	}
	*/
}

float CPhysicsFrictionSnapshot::GetFrictionCoefficient() {
	NOT_IMPLEMENTED;
	return 0;
}

