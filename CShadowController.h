#ifndef CSHADOWCONTROLLER_H
#define CSHADOWCONTROLLER_H

#include "IController.h"

class CPhysicsObject;

float ComputeShadowControllerHL(CPhysicsObject *pObject, const hlshadowcontrol_params_t &params, float secondsToArrival, float dt);

struct shadowcontrol_params_t {
	shadowcontrol_params_t() {lastPosition.setZero();}

	btVector3 targetPosition;
	btQuaternion targetRotation;
	btVector3 maxSpeed;
	btVector3 maxAngular;
	btVector3 lastPosition;
	float dampFactor;
	float teleportDistance;
};

class CShadowController : public IController, public IPhysicsShadowController {
public:
	CShadowController(CPhysicsObject* pObject, bool allowTranslation, bool allowRotation);
	virtual ~CShadowController();
	virtual void Update(const Vector &position, const QAngle &angles, float timeOffset);
	virtual void MaxSpeed(float maxSpeed, float maxAngularSpeed);
	virtual void StepUp(float height);
	virtual void SetTeleportDistance(float teleportDistance);
	virtual bool AllowsTranslation();
	virtual bool AllowsRotation();
	virtual void SetPhysicallyControlled(bool isPhysicallyControlled);
	virtual bool IsPhysicallyControlled();
	virtual void GetLastImpulse(Vector* pOut);
	virtual void UseShadowMaterial(bool bUseShadowMaterial);
	virtual void ObjectMaterialChanged(int materialIndex);
	virtual float GetTargetPosition(Vector* pPositionOut, QAngle* pAnglesOut);
	virtual float GetTeleportDistance();
	virtual void GetMaxSpeed(float* pMaxSpeedOut, float* pMaxAngularSpeedOut);
public:
	virtual void Tick(float deltaTime);
private:
	void AttachObject();
	void DetachObject();

	CPhysicsObject* m_pObject;
	float m_secondsToArrival;
	btVector3 m_currentSpeed;
	float m_savedMass;
	int m_savedMaterialIndex;
	bool m_enable;
	bool m_allowPhysicsMovement;
	bool m_allowPhysicsRotation;
	shadowcontrol_params_t m_shadow;
};

#endif
