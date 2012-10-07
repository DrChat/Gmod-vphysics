#ifndef CPHYSICSMOTIONCONTROLLER_H
#define CPHYSICSMOTIONCONTROLLER_H

#include "IController.h"

class CPhysicsEnvironment;

class CPhysicsMotionController : public IController, public IPhysicsMotionController
{
	public:
										CPhysicsMotionController(IMotionEvent *pHandler, CPhysicsEnvironment *pEnv);
										~CPhysicsMotionController();
		void							SetEventHandler(IMotionEvent *handler);
		void							AttachObject(IPhysicsObject *pObject, bool checkIfAlreadyAttached);
		void							DetachObject(IPhysicsObject *pObject);

		int								CountObjects();
		void							GetObjects(IPhysicsObject **pObjectList);
		void							ClearObjects();
		void							WakeObjects();

		void							SetPriority(priority_t priority);
	public:
		void							Tick(float deltaTime);
	private:
		IMotionEvent *					m_handler;
		CUtlVector<btCollisionObject*>	m_objectList;
		CPhysicsEnvironment *			m_pEnv;

		int								m_priority;
};

IPhysicsMotionController *CreateMotionController(CPhysicsEnvironment *pEnv, IMotionEvent *pHandler);

#endif
