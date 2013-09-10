#include "btParallelCollisionDispatcher.h"

#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

class btProcessOverlapTask : public btIThreadTask {
	public:
		btProcessOverlapTask(btBroadphasePair &pair, const btDispatcherInfo &info, btParallelCollisionDispatcher *pDispatcher): m_pair(pair), m_info(info) {
			m_pDispatcher = pDispatcher;
		}

		void run() {
			btCollisionObject *obj0 = (btCollisionObject *)m_pair.m_pProxy0->m_clientObject;
			btCollisionObject *obj1 = (btCollisionObject *)m_pair.m_pProxy1->m_clientObject;

			btCollisionObjectWrapper obj0Wrap(0, obj0->getCollisionShape(), obj0, obj0->getWorldTransform(), -1, -1);
			btCollisionObjectWrapper obj1Wrap(0, obj1->getCollisionShape(), obj1, obj1->getWorldTransform(), -1, -1);

			if (!m_pair.m_algorithm) {
				m_pair.m_algorithm = m_pDispatcher->findAlgorithm(&obj0Wrap, &obj1Wrap);
			}

			if (m_pair.m_algorithm) {
				btManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap);
				
				if (m_info.m_dispatchFunc == btDispatcherInfo::DISPATCH_DISCRETE) {
					m_pair.m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, m_info, &contactPointResult);
				}
			}
		}

		// Called after run() to destroy this task.
		void destroy() {
			btFree(this);
		}

	private:
		btBroadphasePair &m_pair;
		const btDispatcherInfo &m_info;
		btParallelCollisionDispatcher *m_pDispatcher;
};

btParallelCollisionDispatcher::btParallelCollisionDispatcher(btCollisionConfiguration *pConfiguration, int numThreads) : btCollisionDispatcher(pConfiguration) {
	void *mem = btAlloc(sizeof(btThreadPool));
	m_pThreadPool = new(mem) btThreadPool();

	m_pPoolCritSect = btCreateCriticalSection();
	m_pAlgoPoolSect = btCreateCriticalSection();

	m_pThreadPool->startThreads(numThreads);
}

btParallelCollisionDispatcher::~btParallelCollisionDispatcher() {
	m_pThreadPool->stopThreads();
	m_pThreadPool->~btThreadPool();
	btFree(m_pThreadPool);

	btDeleteCriticalSection(m_pPoolCritSect);
	btDeleteCriticalSection(m_pAlgoPoolSect);
}

btPersistentManifold *btParallelCollisionDispatcher::getNewManifold(const btCollisionObject *ob0, const btCollisionObject *ob1) {
	m_pPoolCritSect->lock();
	btPersistentManifold *ret = btCollisionDispatcher::getNewManifold(ob0, ob1);
	m_pPoolCritSect->unlock();

	return ret;
}

void btParallelCollisionDispatcher::releaseManifold(btPersistentManifold *pManifold) {
	m_pPoolCritSect->lock();
	btCollisionDispatcher::releaseManifold(pManifold);
	m_pPoolCritSect->unlock();
}

void *btParallelCollisionDispatcher::allocateCollisionAlgorithm(int size) {
	m_pAlgoPoolSect->lock();
	void *ret = btCollisionDispatcher::allocateCollisionAlgorithm(size);
	m_pAlgoPoolSect->unlock();

	return ret;
}

void btParallelCollisionDispatcher::freeCollisionAlgorithm(void *ptr) {
	m_pAlgoPoolSect->lock();
	btCollisionDispatcher::freeCollisionAlgorithm(ptr);
	m_pAlgoPoolSect->unlock();
}

class btCollisionPairCallback : public btOverlapCallback {
	public:
		btCollisionPairCallback(const btDispatcherInfo &info, btParallelCollisionDispatcher *pDispatcher): m_info(info) {
			m_pDispatcher = pDispatcher;
		}

		bool processOverlap(btBroadphasePair &pair) {
			btCollisionObject *obj0 = (btCollisionObject *)pair.m_pProxy0->m_clientObject;
			btCollisionObject *obj1 = (btCollisionObject *)pair.m_pProxy1->m_clientObject;

			if (m_pDispatcher->needsCollision(obj0, obj1)) {
				// FIXME: This uses alot of cycles
				void *mem = btAlloc(sizeof(btProcessOverlapTask)); // TODO: Persistent pool or something
				btProcessOverlapTask *task = new(mem) btProcessOverlapTask(pair, m_info, m_pDispatcher);
				m_pDispatcher->getThreadPool()->addTask(task);
			}

			// Always return false (for whatever reason)
			return false;
		}

	private:
		const btDispatcherInfo &m_info;
		btParallelCollisionDispatcher *m_pDispatcher;
};

void btParallelCollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache *pairCache, const btDispatcherInfo &dispatchInfo, btDispatcher *dispatcher) {
	btCollisionPairCallback cb(dispatchInfo, this);
	pairCache->processAllOverlappingPairs(&cb, dispatcher);

	// Wait until the task pool empties out (all threads are finished executing tasks)
	m_pThreadPool->waitIdle();
}

btThreadPool *btParallelCollisionDispatcher::getThreadPool() {
	return m_pThreadPool;
}