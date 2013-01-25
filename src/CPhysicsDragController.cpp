#include "StdAfx.h"

#include "CPhysicsDragController.h"
#include "CPhysicsObject.h"
#include "convert.h"

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

void CPhysicsDragController::RemovePhysicsObject(CPhysicsObject * obj) {
	m_ents.FindAndRemove(obj);
}

void CPhysicsDragController::AddPhysicsObject(CPhysicsObject * obj) {
	if (!m_ents.Find(obj)) {
		m_ents.AddToTail(obj);
	}
}

bool CPhysicsDragController::IsControlling(const CPhysicsObject * obj) const {
	return (m_ents.Find((CPhysicsObject *)obj) != NULL);
}

void CPhysicsDragController::Tick(btScalar dt) {
	int iEntCount = m_ents.Count();
	for (int i = 0; i < iEntCount; i++) {
		CPhysicsObject *object = (CPhysicsObject *)m_ents[i];

		Vector dragLinearFinal(0,0,0);
		AngularImpulse dragAngularFinal(0,0,0);

		Vector vel;
		AngularImpulse ang;
		object->GetVelocity(&vel, &ang);

		btVector3 bull_vel;
		btVector3 bull_angimpulse;

		ConvertPosToBull(vel, bull_vel);
		ConvertAngularImpulseToBull(ang, bull_angimpulse);

		float dragForce = -0.5 * object->GetDragInDirection(&bull_vel) * m_airDensity * dt;
		if (dragForce < -1.0f)
			dragForce = -1.0f;
		
		if (dragForce < 0)
			Vector dragLinearFinal = vel * dragForce;

		float angDragForce = -object->GetAngularDragInDirection(&bull_angimpulse) * m_airDensity * dt;

		if (angDragForce < -1.0f)
			angDragForce = -1.0f;

		if (angDragForce < 0)
			dragAngularFinal = ang * angDragForce;

		object->AddVelocity(&dragLinearFinal, &dragAngularFinal);
	}
}