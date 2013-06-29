#ifndef PHYSICS_H
#define PHYSICS_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

class IPhysicsEnvironment;
class IPhysicsCollisionSet;

class CPhysics : public CTier1AppSystem<IPhysics1> {
	typedef CTier1AppSystem<IPhysics1> BaseClass;
	public:
		~CPhysics();

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
		CUtlVector<IPhysicsCollisionSet *>	m_collisionSets;
};

extern CPhysics g_Physics;

#endif