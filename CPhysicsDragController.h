#ifndef CPHYSICSDRAGCONTROLLER_H
#define CPHYSICSDRAGCONTROLLER_H

class CPhysicsObject;

class CPhysicsDragController
{
public:
	CPhysicsDragController();
	void SetAirDensity(float density);
	float GetAirDensity();

	void AddPhysicsObject(CPhysicsObject *);
	void RemovePhysicsObject(CPhysicsObject *);
	void Tick(btScalar dt);
	bool IsControlling(const CPhysicsObject *) const;

private:
	float m_airDensity;
	CUtlVector<CPhysicsObject*> m_ents;
};

#endif