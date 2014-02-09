#ifndef BT_PARALLELCOLLISIONDISPATCHER_H
#define BT_PARALLELCOLLISIONDISPATCHER_H

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "LinearMath/btPoolAllocator.h"

#include "btThreadPool.h"
#include "btThreading.h"

class btParallelCollisionDispatcher;
struct btDispatcherInfo;

// Internal class used by the parallel collision dispatcher.
class btProcessOverlapTask : public btIThreadTask {
	public:
		btProcessOverlapTask(btBroadphasePair *pair, const btDispatcherInfo *info, btParallelCollisionDispatcher *pDispatcher);
		btProcessOverlapTask();
		void run();

	private:
		btBroadphasePair *m_pair;
		const btDispatcherInfo *m_pInfo;
		btParallelCollisionDispatcher *m_pDispatcher;
};

class btParallelCollisionDispatcher : public btCollisionDispatcher {
	public:
		btParallelCollisionDispatcher(btCollisionConfiguration *pConfiguration, btThreadPool *pThreadPool);
		~btParallelCollisionDispatcher();

		virtual btPersistentManifold *getNewManifold(const btCollisionObject *ob1, const btCollisionObject *ob2);
		virtual void releaseManifold(btPersistentManifold *pManifold);

		virtual	void *allocateCollisionAlgorithm(int size);
		virtual	void freeCollisionAlgorithm(void *ptr);

		virtual btProcessOverlapTask *allocateTask();

		virtual void dispatchAllCollisionPairs(btOverlappingPairCache *pairCache, const btDispatcherInfo &dispatchInfo, btDispatcher *dispatcher);

		btThreadPool *getThreadPool();

	private:
		btAlignedObjectArray<btProcessOverlapTask> m_taskArr;

		btICriticalSection *m_pPoolCritSect;
		btICriticalSection *m_pAlgoPoolSect;
		btThreadPool *		m_pThreadPool;
};

#endif // BT_PARALLELCOLLISIONDISPATCHER_H