#ifndef BT_PARALLELCOLLISIONDISPATCHER_H
#define BT_PARALLELCOLLISIONDISPATCHER_H

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "LinearMath/btPoolAllocator.h"

#include "ThreadPool.h"
#include "Threading.h"

class btParallelCollisionDispatcher : public btCollisionDispatcher {
	public:
		btParallelCollisionDispatcher(btCollisionConfiguration *pConfiguration, int numThreads);
		~btParallelCollisionDispatcher();

		virtual btPersistentManifold *getNewManifold(const btCollisionObject *ob1, const btCollisionObject *ob2);
		virtual void releaseManifold(btPersistentManifold *pManifold);

		virtual	void *allocateCollisionAlgorithm(int size);
		virtual	void freeCollisionAlgorithm(void *ptr);

		virtual void dispatchAllCollisionPairs(btOverlappingPairCache *pairCache, const btDispatcherInfo &dispatchInfo, btDispatcher *dispatcher);

		btThreadPool *getThreadPool();

	private:
		btICriticalSection *m_pPoolCritSect;
		btICriticalSection *m_pAlgoPoolSect;
		btThreadPool *		m_pThreadPool;
};

#endif // BT_PARALLELCOLLISIONDISPATCHER_H