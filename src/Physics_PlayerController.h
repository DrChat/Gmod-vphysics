#ifndef PHYSICS_PLAYERCONTROLLER_H
#define PHYSICS_PLAYERCONTROLLER_H

#include <vphysics/player_controller.h>
#include "IController.h"

class CPhysicsObject;

class CPlayerController : public IController, public IPhysicsPlayerController
{
	public:
		CPlayerController(CPhysicsObject *pObject);
		~CPlayerController();

		void							Update(const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground);
		void							SetEventHandler(IPhysicsPlayerControllerEvent *handler);
		bool							IsInContact();
		void							MaxSpeed(const Vector &maxVelocity);

		void							SetObject(IPhysicsObject *pObject);
		int								GetShadowPosition(Vector *position, QAngle *angles);
		void							StepUp(float height);
		void							Jump();
		void							GetShadowVelocity(Vector *velocity);
		IPhysicsObject *				GetObject();
		void							GetLastImpulse(Vector *pOut);

		void							SetPushMassLimit(float maxPushMass);
		void							SetPushSpeedLimit(float maxPushSpeed);
	
		float							GetPushMassLimit();
		float							GetPushSpeedLimit();
		bool							WasFrozen();
	
		// Unexposed functions
	public:
		void							Tick(float deltaTime);

	private:
		void							AttachObject();
		void							DetachObject();
		bool							TryTeleportObject();

		bool							m_enable;
		bool							m_onground;
		CPhysicsObject *				m_pGround;
		CPhysicsObject *				m_pObject;
		btVector3						m_saveRot;
		IPhysicsPlayerControllerEvent *	m_handler;
		float							m_maxDeltaPosition;
		float							m_dampFactor;
		float							m_secondsToArrival;
		btVector3						m_targetPosition;
		btVector3						m_maxSpeed;
		btVector3						m_currentSpeed;
		btVector3						m_lastImpulse;
		float							m_PushMassLimit;
		float							m_PushSpeedLimit;

		int								m_iTicksSinceUpdate;
};

void ComputeController(btVector3 &currentSpeed, const btVector3 &delta, const btVector3 &maxSpeed, float scaleDelta, float damping);

CPlayerController *CreatePlayerController(IPhysicsObject *pObject);

#endif // PHYSICS_PLAYERCONTROLLER_H
