#ifndef CPHYSICS_H
#define CPHYSICS_H

class CPhysicsCollisionSet;

class CPhysics : public CTier1AppSystem<IPhysics> {
	typedef CTier1AppSystem<IPhysics> BaseClass;
	public:
		virtual void *						QueryInterface(const char *pInterfaceName);
		virtual InitReturnVal_t				Init();
		virtual void						Shutdown();

		virtual	IPhysicsEnvironment *		CreateEnvironment();
		virtual void						DestroyEnvironment(IPhysicsEnvironment*);
		virtual IPhysicsEnvironment *		GetActiveEnvironmentByIndex(int index);

		virtual IPhysicsObjectPairHash *	CreateObjectPairHash();
		virtual void						DestroyObjectPairHash(IPhysicsObjectPairHash *pHash);

		virtual IPhysicsCollisionSet *		FindOrCreateCollisionSet( unsigned int id, int maxElementCount );
		virtual IPhysicsCollisionSet *		FindCollisionSet( unsigned int id );
		virtual void						DestroyAllCollisionSets();
	private:
		CUtlVector<IPhysicsEnvironment *>	m_envList;
		CUtlVector<CPhysicsCollisionSet *>	m_collisionSets;
		CSysModule *						m_valvephysics;
};

#endif