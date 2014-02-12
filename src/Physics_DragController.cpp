#include "StdAfx.h"

#include "Physics_DragController.h"
#include "Physics_Object.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/******************************
* CLASS CPhysicsDragController
******************************/

CPhysicsDragController::CPhysicsDragController() {
	m_airDensity = 2.f; // default
}

void CPhysicsDragController::SetAirDensity(float d) {
	// Density is kg/in^3 from HL
	m_airDensity = d / CUBIC_METERS_PER_CUBIC_INCH;
}

float CPhysicsDragController::GetAirDensity() {
	return m_airDensity * CUBIC_METERS_PER_CUBIC_INCH;
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
	for (int i = 0; i < m_ents.Count(); i++) {
		CPhysicsObject *pObject = (CPhysicsObject *)m_ents[i];
		btRigidBody *body = pObject->GetObject();

		btVector3 vel(0, 0, 0);
		btVector3 ang(0, 0, 0);

		//------------------
		// LINEAR DRAG
		//------------------
		float dragForce = -1 * pObject->GetDragInDirection(body->getLinearVelocity()) * m_airDensity * dt;
		if (dragForce < -1.0f)
			dragForce = -1.0f;

		// If the drag force actually drags
		if (dragForce < 0)
			vel = body->getLinearVelocity() * dragForce;

		body->setLinearVelocity(body->getLinearVelocity() + vel);

		//------------------
		// ANGULAR DRAG
		//------------------
		float angDragForce = -1 * pObject->GetAngularDragInDirection(body->getAngularVelocity()) * m_airDensity * dt;
		if (angDragForce < -1.0f)
			angDragForce = -1.0f;

		// If the drag force actually drags
		if (angDragForce < 0)
			ang = body->getAngularVelocity() * angDragForce;

		body->setAngularVelocity(body->getAngularVelocity() + ang);
	}
}