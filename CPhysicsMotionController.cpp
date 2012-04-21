#include "StdAfx.h"

#include "CPhysicsMotionController.h"

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
	NOT_IMPLEMENTED;
}

void CPhysicsMotionController::DetachObject(IPhysicsObject* pObject) {
	NOT_IMPLEMENTED;
}

int CPhysicsMotionController::CountObjects() {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsMotionController::GetObjects(IPhysicsObject** pObjectList) {
	NOT_IMPLEMENTED;
}

void CPhysicsMotionController::ClearObjects() {
	NOT_IMPLEMENTED;
}

void CPhysicsMotionController::WakeObjects() {
	NOT_IMPLEMENTED;
}

void CPhysicsMotionController::SetPriority(priority_t priority) {
	NOT_IMPLEMENTED;
}
