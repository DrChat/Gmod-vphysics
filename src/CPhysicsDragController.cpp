#include "StdAfx.h"

#include "CPhysicsDragController.h"
#include "CPhysicsObject.h"
#include "convert.h"

#include "tier0/vprof.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/******************************
* CLASS CPhysicsDragController
******************************/

CPhysicsDragController::CPhysicsDragController() {
	m_airDensity = 2; // default
}

void CPhysicsDragController::SetAirDensity(float d) {
	m_airDensity = d;
}

float CPhysicsDragController::GetAirDensity() {
	return m_airDensity;
}

void CPhysicsDragController::RemovePhysicsObject(CPhysicsObject *obj) {
	m_ents.FindAndRemove(obj);
}

void CPhysicsDragController::AddPhysicsObject(CPhysicsObject *obj) {
	if (!IsControlling(obj)) {
		m_ents.AddToTail(obj);
	}
}

bool CPhysicsDragController::IsControlling(const CPhysicsObject *obj) const {
	return m_ents.Find((CPhysicsObject *)obj) != -1;
}

void CPhysicsDragController::Tick(btScalar dt) {
	VPROF_BUDGET("CPhysicsDragController::Tick", VPROF_BUDGETGROUP_PHYSICS);

	int iEntCount = m_ents.Count();
	for (int i = 0; i < iEntCount; i++) {
		CPhysicsObject *pObject = (CPhysicsObject *)m_ents[i];
		btRigidBody *body = pObject->GetObject();

		btVector3 vel(0, 0, 0);
		btVector3 ang(0, 0, 0);

		//------------------
		// LINEAR DRAG
		//------------------
		float dragForce = -0.5f * pObject->GetDragInDirection(body->getLinearVelocity()) * m_airDensity * dt;
		if (dragForce < -1.0f)
			dragForce = -1.0f;

		if (dragForce < 0)
			vel = body->getLinearVelocity() * dragForce;

		body->setLinearVelocity(body->getLinearVelocity() + vel);

		//------------------
		// ANGULAR DRAG
		//------------------
		float angDragForce = -pObject->GetAngularDragInDirection(body->getAngularVelocity()) * m_airDensity * dt;
		if (angDragForce < -1.0f)
			angDragForce = -1.0f;

		if (angDragForce < 0)
			ang = body->getAngularVelocity() * angDragForce;

		body->setAngularVelocity(body->getAngularVelocity() + ang);
	}
}