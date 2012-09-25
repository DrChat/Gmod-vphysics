#ifndef CPHYSICSCOLLISIONSET_H
#define CPHYSICSCOLLISIONSET_H
// FIXME: Slacking out on the speed of this one

class CPhysicsCollisionSet : public IPhysicsCollisionSet
{
public:
					CPhysicsCollisionSet(int iMaxEntries);
					~CPhysicsCollisionSet();

	virtual void	EnableCollisions(int index0, int index1);
	virtual void	DisableCollisions(int index0, int index1);

	virtual bool	ShouldCollide(int index0, int index1);
private:
	int				m_iMaxEntries;
};

#endif // CPHYSICSCOLLISIONSET_H