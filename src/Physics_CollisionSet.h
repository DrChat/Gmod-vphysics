#ifndef CPHYSICSCOLLISIONSET_H
#define CPHYSICSCOLLISIONSET_H

class CPhysicsCollisionSet : public IPhysicsCollisionSet
{
	public:
						CPhysicsCollisionSet(int iMaxEntries);
						~CPhysicsCollisionSet();

		void			EnableCollisions(int index0, int index1);
		void			DisableCollisions(int index0, int index1);

		bool			ShouldCollide(int index0, int index1);
	private:
		int				m_iMaxEntries;
};

#endif // CPHYSICSCOLLISIONSET_H