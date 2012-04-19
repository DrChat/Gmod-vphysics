#ifndef CPLAYERCONTROLLER
#define CPLAYERCONTROLLER

class CPhysicsObject;

class CPlayerController : public btDefaultMotionState, public IPhysicsPlayerController {
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
private:
	CPhysicsObject* m_pObject;
	IPhysicsPlayerControllerEvent* m_handler;
	float m_PushMassLimit;
	float m_PushSpeedLimit;
};

#endif
