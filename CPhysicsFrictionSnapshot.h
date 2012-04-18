#ifndef CPHYSICSFRICTIONSNAPSHOT_H
#define CPHYSICSFRICTIONSNAPSHOT_H

class CPhysicsFrictionSnapshot : public IPhysicsFrictionSnapshot {
public:
	CPhysicsFrictionSnapshot();
	virtual ~CPhysicsFrictionSnapshot();
	virtual bool IsValid();

	virtual IPhysicsObject* GetObject(int index);
	virtual int GetMaterial(int index);

	virtual void GetContactPoint(Vector& out);
	
	virtual void GetSurfaceNormal(Vector& out);
	virtual float GetNormalForce();
	virtual float GetEnergyAbsorbed();

	virtual void RecomputeFriction();
	virtual void ClearFrictionForce();

	virtual void MarkContactForDelete();
	virtual void DeleteAllMarkedContacts(bool wakeObjects);

	virtual void NextFrictionData();
	virtual float GetFrictionCoefficient();
};

#endif
