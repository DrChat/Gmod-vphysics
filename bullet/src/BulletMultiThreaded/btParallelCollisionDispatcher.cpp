/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Dr. Chat / Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <BulletCollision/BroadphaseCollision/btOverlappingPairCache.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>

#include "btParallelCollisionDispatcher.h"

// Internal task ids
enum TaskIds
{
	TASK_PROCESS_COLLISION = 0,
};

class btParallelCollisionPairCallback : public btOverlapCallback
{
	public:
		btParallelCollisionPairCallback(const btDispatcherInfo &dispatchInfo, btCollisionDispatcher *dispatcher) :
			m_dispatchInfo(dispatchInfo),
			m_dispatcher(dispatcher)
		{
		}

		virtual ~btParallelCollisionPairCallback() {}

		virtual bool processOverlap(btBroadphasePair &pair)
		{
			(*m_dispatcher->getNearCallback())(pair, *m_dispatcher, m_dispatchInfo);

			// return false so this pair isn't removed from the full pair list.
			return false;
		}

	private:
		const btDispatcherInfo &m_dispatchInfo;
		btCollisionDispatcher *	m_dispatcher;
};

struct btProcessCollisionInfo
{
	// FIXME: Doesn't need to be an aligned array.
	btAlignedObjectArray<btBroadphasePair *> m_pairs;
	btCollisionDispatcher *	m_dispatcher;
	const btDispatcherInfo *m_dispatchInfo;
};

void ProcessCollision(btProcessCollisionInfo *info)
{
	for (uint32_t i = 0; i < info->m_pairs.size(); i++)
	{
		btBroadphasePair *pair = info->m_pairs[i];
		btCollisionObject *colObj0 = (btCollisionObject *)pair->m_pProxy0->m_clientObject;
		btCollisionObject *colObj1 = (btCollisionObject *)pair->m_pProxy1->m_clientObject;

		btCollisionObjectWrapper obj0Wrap(0, colObj0->getCollisionShape(), colObj0, colObj0->getWorldTransform(), -1, -1);
		btCollisionObjectWrapper obj1Wrap(0, colObj1->getCollisionShape(), colObj1, colObj1->getWorldTransform(), -1, -1);

		if (pair->m_algorithm)
		{
			btManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap);
			
			if (info->m_dispatchInfo->m_dispatchFunc == btDispatcherInfo::DISPATCH_DISCRETE)
			{
				// discrete collision detection query
				pair->m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, *info->m_dispatchInfo, &contactPointResult);
			}
			else
			{
				// continuous collision detection query, time of impact (toi)
				btScalar toi = pair->m_algorithm->calculateTimeOfImpact(colObj0, colObj1, *info->m_dispatchInfo, &contactPointResult);
				if (info->m_dispatchInfo->m_timeOfImpact > toi)
					info->m_dispatchInfo->m_timeOfImpact = toi;
			}
		}
	}
}

// Thread function
int DispatchThreadFunc(int taskId, void *pArg)
{
	switch (taskId)
	{
		case TASK_PROCESS_COLLISION:
			ProcessCollision((btProcessCollisionInfo *)pArg);
			break;
	}

	return 0;
}

btParallelCollisionDispatcher::btParallelCollisionDispatcher(btCollisionConfiguration *collisionConfiguration, btIThreadPool *threadPool, uint32_t numThreads) :
	btCollisionDispatcher(collisionConfiguration),
	m_pThreadPool(threadPool)
{
	btThreadPoolInfo pi("ParallelCollisionDispatch", DispatchThreadFunc, numThreads);
	m_pThreadPool->startThreads(pi);

	m_pCriticalSection = m_pThreadPool->createCriticalSection();

	btCollisionDispatcher::setNearCallback(btParallelCollisionDispatcher::defaultParallelNearCallback);
}

btParallelCollisionDispatcher::~btParallelCollisionDispatcher()
{
	m_pThreadPool->destroyCriticalSection(m_pCriticalSection);
	m_pThreadPool->stopThreads();
}

btPersistentManifold *btParallelCollisionDispatcher::getNewManifold(const btCollisionObject *b0, const btCollisionObject *b1)
{
	// This may be called from multiple threads, so we have to synchronize this.
	m_pCriticalSection->lock();
	btPersistentManifold *pManifold = btCollisionDispatcher::getNewManifold(b0, b1);
	m_pCriticalSection->unlock();

	return pManifold;
}

void btParallelCollisionDispatcher::releaseManifold(btPersistentManifold *manifold)
{
	// This may be called from multiple threads, so we have to synchronize this.
	m_pCriticalSection->lock();
	btCollisionDispatcher::releaseManifold(manifold);
	m_pCriticalSection->unlock();
}

void btParallelCollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache *pairCache, const btDispatcherInfo &dispatchInfo, btDispatcher *dispatcher)
{
	btParallelCollisionPairCallback cb(dispatchInfo, this);
	pairCache->processAllOverlappingPairs(&cb, dispatcher);

	// We've sorted out all the algorithms. Now let's start dividing the workload up for our thread pool.
	int totalNumPairs = pairCache->getNumOverlappingPairs();
	if (totalNumPairs > 0)
	{
		// Sort out any pairs that don't need collision first.
		btAlignedObjectArray<btBroadphasePair *> pairs;
		btBroadphasePair *pairArr = pairCache->getOverlappingPairArrayPtr();
		for (uint32_t i = 0; i < totalNumPairs; i++)
		{
			btBroadphasePair &collisionPair = pairArr[i];
			btCollisionObject *colObj0 = (btCollisionObject *)collisionPair.m_pProxy0->m_clientObject;
			btCollisionObject *colObj1 = (btCollisionObject *)collisionPair.m_pProxy1->m_clientObject;

			if (dispatcher->needsCollision(colObj0, colObj1))
			{
				pairs.push_back(&collisionPair);
			}
		}

		// FIXME: Will need to account for any pairs that are skipped due to rounding down.
		int pairsPerThread = pairs.size() / m_pThreadPool->getNumThreads();

		for (uint32_t i = 0; i < m_pThreadPool->getNumThreads(); i++)
		{

		}
	}
}

void btParallelCollisionDispatcher::defaultParallelNearCallback(btBroadphasePair &collisionPair, btCollisionDispatcher &dispatcher, const btDispatcherInfo &dispatchInfo)
{
	btCollisionObject *colObj0 = (btCollisionObject *)collisionPair.m_pProxy0->m_clientObject;
	btCollisionObject *colObj1 = (btCollisionObject *)collisionPair.m_pProxy1->m_clientObject;

	if (dispatcher.needsCollision(colObj0, colObj1))
	{
		btCollisionObjectWrapper obj0Wrap(0, colObj0->getCollisionShape(), colObj0, colObj0->getWorldTransform(), -1, -1);
		btCollisionObjectWrapper obj1Wrap(0, colObj1->getCollisionShape(), colObj1, colObj1->getWorldTransform(), -1, -1);

		// dispatcher will keep algorithms persistent in the collision pair
		if (!collisionPair.m_algorithm)
		{
			collisionPair.m_algorithm = dispatcher.findAlgorithm(&obj0Wrap, &obj1Wrap);
		}

		// The collision won't be processed here!
		/*
		if (collisionPair.m_algorithm)
		{
			btManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap);
			
			if (dispatchInfo.m_dispatchFunc == btDispatcherInfo::DISPATCH_DISCRETE)
			{
				// discrete collision detection query
				collisionPair.m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, dispatchInfo, &contactPointResult);
			}
			else
			{
				//continuous collision detection query, time of impact (toi)
				btScalar toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0, colObj1, dispatchInfo, &contactPointResult);
				if (dispatchInfo.m_timeOfImpact > toi)
					dispatchInfo.m_timeOfImpact = toi;

			}
		}
		*/
	}
}