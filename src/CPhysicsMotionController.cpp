#include "StdAfx.h"

#include "CPhysicsMotionController.h"
#include "CPhysicsObject.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

IPhysicsMotionController *CreateMotionController(CPhysicsEnvironment *pEnv, IMotionEvent *pHandler) {
	if (!pHandler) return NULL;
	return new CPhysicsMotionController(pHandler, pEnv);
}

/***********************************
* CLASS CPhysicsMotionController
***********************************/

CPhysicsMotionController::CPhysicsMotionController(IMotionEvent *pHandler, CPhysicsEnvironment *pEnv) {
	m_handler = pHandler;
	m_pEnv = pEnv;

	SetPriority(MEDIUM_PRIORITY);
}

CPhysicsMotionController::~CPhysicsMotionController() {

}

void CPhysicsMotionController::Tick(float deltaTime) {
	if (!m_handler) return;
	for (int i = 0; i < m_objectList.Count(); i++) {
		Vector speed;
		AngularImpulse rot;
		btVector3 bullSpeed, bullRot;

		btRigidBody *body = btRigidBody::upcast(m_objectList[i]);
		IPhysicsObject *pObject = (IPhysicsObject *)body->getUserPointer();
		IMotionEvent::simresult_e ret = m_handler->Simulate(this, pObject, deltaTime, speed, rot);
		switch(ret) {
			case IMotionEvent::SIM_NOTHING: {
				break;
			}
			case IMotionEvent::SIM_LOCAL_ACCELERATION: {
				ConvertForceImpulseToBull(speed, bullSpeed);
				ConvertAngularImpulseToBull(rot, bullRot);

				btTransform transform;
				((btMassCenterMotionState *)body->getMotionState())->getGraphicTransform(transform);
				bullSpeed = transform.getBasis()*bullSpeed;

				body->setLinearVelocity(body->getLinearVelocity() + bullSpeed * deltaTime);
				body->setAngularVelocity(body->getAngularVelocity() + bullRot * deltaTime);
				break;
			}
			case IMotionEvent::SIM_LOCAL_FORCE: {
				ConvertForceImpulseToBull(speed, bullSpeed);
				ConvertAngularImpulseToBull(rot, bullRot);

				btTransform transform;
				((btMassCenterMotionState *)body->getMotionState())->getGraphicTransform(transform);
				bullSpeed = transform.getBasis()*bullSpeed;

				body->applyCentralForce(bullSpeed * deltaTime);
				body->applyTorque(bullRot * deltaTime);
				break;
			}
			case IMotionEvent::SIM_GLOBAL_ACCELERATION: {
				ConvertForceImpulseToBull(speed, bullSpeed);
				ConvertAngularImpulseToBull(rot, bullRot);
				body->setLinearVelocity(body->getLinearVelocity() + bullSpeed * deltaTime);
				body->setAngularVelocity(body->getAngularVelocity() + bullRot * deltaTime);
				break;
			}
			case IMotionEvent::SIM_GLOBAL_FORCE: {
				ConvertForceImpulseToBull(speed, bullSpeed);
				ConvertAngularImpulseToBull(rot, bullRot);
				body->applyCentralForce(bullSpeed * deltaTime);
				body->applyTorque(bullRot * deltaTime);
				break;
			}
		}
	}
}

void CPhysicsMotionController::SetEventHandler(IMotionEvent *handler) {
	m_handler = handler;
}

void CPhysicsMotionController::AttachObject(IPhysicsObject *pObject, bool checkIfAlreadyAttached) {
	Assert(pObject);
	if (!pObject || pObject->IsStatic()) return;

	CPhysicsObject *pPhys = (CPhysicsObject *)pObject;
	btRigidBody *body = pPhys->GetObject();
	m_objectList.AddToTail(body);
}

void CPhysicsMotionController::DetachObject(IPhysicsObject *pObject) {
	CPhysicsObject *pPhys = (CPhysicsObject *)pObject;
	btRigidBody *body = pPhys->GetObject();

	int index = m_objectList.Find(body);
	if (!m_objectList.IsValidIndex(index)) return;
	m_objectList.Remove(index);
}

int CPhysicsMotionController::CountObjects() {
	return m_objectList.Count();
}

void CPhysicsMotionController::GetObjects(IPhysicsObject **pObjectList) {
	for (int i = 0; i < m_objectList.Count(); i++) {
		pObjectList[i] = (IPhysicsObject *)m_objectList[i]->getUserPointer();
	}
}

void CPhysicsMotionController::ClearObjects() {
	NOT_IMPLEMENTED
}

void CPhysicsMotionController::WakeObjects() {
	for (int i = 0; i < m_objectList.Count(); i++) {
		m_objectList[i]->setActivationState(ACTIVE_TAG);
	}
}

void CPhysicsMotionController::SetPriority(priority_t priority) {
	switch (priority) {
		case LOW_PRIORITY:
			break;
		default:
		case MEDIUM_PRIORITY:
			break;
		case HIGH_PRIORITY:
			break;
	}

	NOT_IMPLEMENTED
}
