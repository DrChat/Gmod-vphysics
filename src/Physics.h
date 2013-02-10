#ifndef CPHYSICS_H
#define CPHYSICS_H

class CPhysicsCollisionSet;

class CPhysics : public CTier1AppSystem<IPhysics> {
	typedef CTier1AppSystem<IPhysics> BaseClass;
	public:
		void *						QueryInterface(const char *pInterfaceName);
		InitReturnVal_t				Init();
		void						Shutdown();

		IPhysicsEnvironment *		CreateEnvironment();
		void						DestroyEnvironment(IPhysicsEnvironment *pEnv);
		IPhysicsEnvironment *		GetActiveEnvironmentByIndex(int index);
		int							GetNumActiveEnvironments();

		IPhysicsObjectPairHash *	CreateObjectPairHash();
		void						DestroyObjectPairHash(IPhysicsObjectPairHash *pHash);

		IPhysicsCollisionSet *		FindOrCreateCollisionSet(unsigned int id, int maxElementCount);
		IPhysicsCollisionSet *		FindCollisionSet(unsigned int id);
		void						DestroyAllCollisionSets();
	private:
		CUtlVector<IPhysicsEnvironment *>	m_envList;
		CUtlVector<CPhysicsCollisionSet *>	m_collisionSets;
};

#endif