#ifndef CPHYSICSFRICTIONSNAPSHOT_H
#define CPHYSICSFRICTIONSNAPSHOT_H

class CPhysicsObject;

class CPhysicsFrictionSnapshot : public IPhysicsFrictionSnapshot {
public:
	CPhysicsFrictionSnapshot(CPhysicsObject* pObject);
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
private:
	CUtlVector<btPersistentManifold*> m_manifolds;
	CPhysicsObject* m_pObject;
	int m_manifold;
	int m_contactpoint;
};

#endif
