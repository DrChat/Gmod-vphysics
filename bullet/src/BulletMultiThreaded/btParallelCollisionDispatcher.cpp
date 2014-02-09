#include "btParallelCollisionDispatcher.h"

#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

btProcessOverlapTask::btProcessOverlapTask() {
	m_pDispatcher = NULL;
	m_pInfo = NULL;
	m_pair = NULL;
}

btProcessOverlapTask::btProcessOverlapTask(btBroadphasePair *pair, const btDispatcherInfo *info, btParallelCollisionDispatcher *pDispatcher) {
	m_pDispatcher = pDispatcher;
	m_pInfo = info;
	m_pair = pair;
}

void btProcessOverlapTask::run() {
	btCollisionObject *obj0 = (btCollisionObject *)m_pair->m_pProxy0->m_clientObject;
	btCollisionObject *obj1 = (btCollisionObject *)m_pair->m_pProxy1->m_clientObject;

	btCollisionObjectWrapper obj0Wrap(0, obj0->getCollisionShape(), obj0, obj0->getWorldTransform(), -1, -1);
	btCollisionObjectWrapper obj1Wrap(0, obj1->getCollisionShape(), obj1, obj1->getWorldTransform(), -1, -1);

	if (!m_pair->m_algorithm) {
		m_pair->m_algorithm = m_pDispatcher->findAlgorithm(&obj0Wrap, &obj1Wrap);
	}

	if (m_pair->m_algorithm) {
		btManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap);

		if (m_pInfo->m_dispatchFunc == btDispatcherInfo::DISPATCH_DISCRETE) {
			m_pair->m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, *m_pInfo, &contactPointResult);
		}
		else {
			// Continuous dispatch
			m_pair->m_algorithm->calculateTimeOfImpact(obj0, obj1, *m_pInfo, &contactPointResult);
		}
	}
}

btParallelCollisionDispatcher::btParallelCollisionDispatcher(btCollisionConfiguration *pConfiguration, btThreadPool *pThreadPool) : btCollisionDispatcher(pConfiguration) {
	m_pThreadPool = pThreadPool;
	m_taskArr.reserve(4096);

	m_pPoolCritSect = btCreateCriticalSection();
	m_pAlgoPoolSect = btCreateCriticalSection();
}

btParallelCollisionDispatcher::~btParallelCollisionDispatcher() {
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

btProcessOverlapTask *btParallelCollisionDispatcher::allocateTask() {
	return &m_taskArr.expandNonInitializing();
}

class btCollisionPairCallback : public btOverlapCallback {
	public:
		btCollisionPairCallback(const btDispatcherInfo &info, btParallelCollisionDispatcher *pDispatcher) {
			m_pDispatcher = pDispatcher;
			m_pInfo = &info;
		}

		bool processOverlap(btBroadphasePair &pair) {
			btCollisionObject *obj0 = (btCollisionObject *)pair.m_pProxy0->m_clientObject;
			btCollisionObject *obj1 = (btCollisionObject *)pair.m_pProxy1->m_clientObject;

			if (m_pDispatcher->needsCollision(obj0, obj1)) {
				btProcessOverlapTask *task = new(m_pDispatcher->allocateTask()) btProcessOverlapTask(&pair, m_pInfo, m_pDispatcher);
				m_pDispatcher->getThreadPool()->addTask(task);
			}

			// Always return false (for whatever reason)
			return false;
		}

	private:
		const btDispatcherInfo *m_pInfo;
		btParallelCollisionDispatcher *m_pDispatcher;
};

void btParallelCollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache *pairCache, const btDispatcherInfo &dispatchInfo, btDispatcher *dispatcher) {
	btCollisionPairCallback cb(dispatchInfo, this);
	pairCache->processAllOverlappingPairs(&cb, dispatcher);

	m_pThreadPool->runTasks();

	// Wait until the task pool empties out (all threads are finished executing tasks)
	m_pThreadPool->waitIdle();

	// Destroy all the tasks (without resizing the array's memory to 0)
	m_taskArr.resize(0);
}

btThreadPool *btParallelCollisionDispatcher::getThreadPool() {
	return m_pThreadPool;
}