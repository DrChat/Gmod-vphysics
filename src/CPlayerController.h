#ifndef CPLAYERCONTROLLER
#define CPLAYERCONTROLLER

#include "IController.h"

class CPhysicsObject;

void ComputeController(btVector3 &currentSpeed, const btVector3 &delta, const btVector3 &maxSpeed, float scaleDelta, float damping);

class CPlayerController : public IController, public IPhysicsPlayerController {
public:
	CPlayerController(CPhysicsObject* pObject);
	virtual ~CPlayerController();
	virtual void Update(const Vector& position, const Vector& velocity, float secondsToArrival, bool onground, IPhysicsObject* ground);
	virtual void SetEventHandler(IPhysicsPlayerControllerEvent* handler);
	virtual bool IsInContact();
	virtual void MaxSpeed(const Vector& maxVelocity);

	virtual void SetObject(IPhysicsObject* pObject);
	virtual int GetShadowPosition( Vector* position, QAngle* angles );
	virtual void StepUp(float height);
	virtual void Jump();
	virtual void GetShadowVelocity(Vector* velocity);
	virtual IPhysicsObject* GetObject();
	virtual void GetLastImpulse(Vector* pOut);

	virtual void SetPushMassLimit(float maxPushMass);
	virtual void SetPushSpeedLimit(float maxPushSpeed);
	
	virtual float GetPushMassLimit();
	virtual float GetPushSpeedLimit();
	virtual bool WasFrozen();
public:
	void Tick(float deltaTime);
private:
	void AttachObject();
	void DetachObject();
	int TryTeleportObject();
	bool m_enable;
	bool m_onground;
	CPhysicsObject* m_pObject;
	btScalar m_saveRot;
	IPhysicsPlayerControllerEvent* m_handler;
	float m_maxDeltaPosition;
	float m_dampFactor;
	btVector3 m_targetPosition;
	btVector3 m_maxSpeed;
	btVector3 m_currentSpeed;
	float m_PushMassLimit;
	float m_PushSpeedLimit;
};

#endif
