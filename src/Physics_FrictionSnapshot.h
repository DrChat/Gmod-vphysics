#ifndef CPHYSICSFRICTIONSNAPSHOT_H
#define CPHYSICSFRICTIONSNAPSHOT_H

class CPhysicsObject;
class btPersistentManifold;

class CPhysicsFrictionSnapshot : public IPhysicsFrictionSnapshot
{
	public:
											CPhysicsFrictionSnapshot(CPhysicsObject *pObject);
											~CPhysicsFrictionSnapshot();
		bool								IsValid();

		IPhysicsObject *					GetObject(int index);
		int									GetMaterial(int index);

		void								GetContactPoint(Vector &out);
	
		void								GetSurfaceNormal(Vector &out);
		float								GetNormalForce();
		float								GetEnergyAbsorbed();

		void								RecomputeFriction();
		void								ClearFrictionForce();

		void								MarkContactForDelete();
		void								DeleteAllMarkedContacts(bool wakeObjects);

		void								NextFrictionData();
		float								GetFrictionCoefficient();
	private:
		CUtlVector<btPersistentManifold *>	m_manifolds;
		CPhysicsObject *					m_pObject;
		int									m_iCurManifold;
		int									m_iCurContactPoint;
};

CPhysicsFrictionSnapshot *CreateFrictionSnapshot(CPhysicsObject *pObject);

#endif
