#ifndef CSHADOWCONTROLLER_H
#define CSHADOWCONTROLLER_H

class CPhysicsObject;

class CShadowController : public IPhysicsShadowController {
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
private:
	CPhysicsObject* m_pObject;
};

#endif
