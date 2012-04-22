#include "StdAfx.h"

#include "CPhysicsMotionController.h"
#include "CPhysicsObject.h"

IPhysicsMotionController *CreateMotionController(CPhysicsEnvironment *pEnv, IMotionEvent *pHandler) {
	if (!pHandler) return NULL;
	return new CPhysicsMotionController(pHandler, pEnv);
}

CPhysicsMotionController::CPhysicsMotionController(IMotionEvent *pHandler, CPhysicsEnvironment *pEnv) {
	m_handler = pHandler;
	m_pEnv = pEnv;
}

CPhysicsMotionController::~CPhysicsMotionController() {

}

void CPhysicsMotionController::SetEventHandler(IMotionEvent* handler) {
	m_handler = handler;
}

void CPhysicsMotionController::AttachObject(IPhysicsObject* pObject, bool checkIfAlreadyAttached) {
	Assert(pObject);
	if (!pObject || pObject->IsStatic()) return;

	CPhysicsObject *pPhys = (CPhysicsObject*)pObject;
	btRigidBody* body = pPhys->GetObject();
	m_objectList.AddToTail(body);
}

void CPhysicsMotionController::DetachObject(IPhysicsObject* pObject) {
	CPhysicsObject *pPhys = (CPhysicsObject*)pObject;
	btRigidBody* body = pPhys->GetObject();

	int index = m_objectList.Find(body);
	if (!m_objectList.IsValidIndex(index)) return;
	m_objectList.Remove(index);
}

int CPhysicsMotionController::CountObjects() {
	return m_objectList.Count();
}

void CPhysicsMotionController::GetObjects(IPhysicsObject** pObjectList) {
	for (int i = 0; i < m_objectList.Count(); i++) {
		pObjectList[i] = (IPhysicsObject*)m_objectList[i]->getUserPointer();
	}
}

void CPhysicsMotionController::ClearObjects() {
	NOT_IMPLEMENTED;
}

void CPhysicsMotionController::WakeObjects() {
	for (int i = 0; i < m_objectList.Count(); i++) {
		m_objectList[i]->setActivationState(ACTIVE_TAG);
	}
}

void CPhysicsMotionController::SetPriority(priority_t priority) {
	NOT_IMPLEMENTED;
}
