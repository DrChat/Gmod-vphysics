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

#ifndef BT_PARALLEL_COLLISION_DISPATCHER_H
#define BT_PARALLEL_COLLISION_DISPATCHER_H

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "btIThreadPool.h"

class btParallelCollisionDispatcher : public btCollisionDispatcher
{
	public:
		btParallelCollisionDispatcher(btCollisionConfiguration *collisionConfiguration, btIThreadPool *threadPool, uint32_t numThreads = 4);
		~btParallelCollisionDispatcher();

		btPersistentManifold *getNewManifold(const btCollisionObject *b0, const btCollisionObject *b1);
		void releaseManifold(btPersistentManifold *manifold);

		void dispatchAllCollisionPairs(btOverlappingPairCache *pairCache, const btDispatcherInfo &dispatchInfo, btDispatcher *dispatcher);

		static void defaultParallelNearCallback(btBroadphasePair &collisionPair, btCollisionDispatcher &dispatcher, const btDispatcherInfo &dispatchInfo);

	private:
		btIThreadPool *		m_pThreadPool;
		btCriticalSection *	m_pCriticalSection;
};

#endif // BT_PARALLEL_COLLISION_DISPATCHER_H