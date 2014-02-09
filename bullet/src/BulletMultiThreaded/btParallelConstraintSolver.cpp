/*
   Copyright (C) 2010 Sony Computer Entertainment Inc.
   All rights reserved.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/


#include "btParallelConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "LinearMath/btPoolAllocator.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

#include "btThreadPool.h"
#include "btThreading.h"

#include "LinearMath/btDefines.h"
#include "LinearMath/btScalar.h"

btParallelConstraintSolver::btParallelConstraintSolver(btThreadPool *pThreadPool) {
	m_pThreadPool = pThreadPool;

	void *taskPoolMem = btAlloc(sizeof(btPoolAllocator));
	//m_pTaskPool = new(taskPoolMem) btPoolAllocator(sizeof(btSolveGroupTask), 4096);
	m_pTaskPoolSect = btCreateCriticalSection();
}

btParallelConstraintSolver::~btParallelConstraintSolver() {
	btDeleteCriticalSection(m_pTaskPoolSect);
	m_pTaskPool->~btPoolAllocator();
	btFree(m_pTaskPool);
}

void *btParallelConstraintSolver::allocateTask(int size) {
	m_pTaskPoolSect->lock();
	void *ret = NULL;
	
	if (m_pTaskPool->getFreeCount() > 0) {
		ret = m_pTaskPool->allocate(size);
	} else {
		ret = btAlloc(size);
	}
	m_pTaskPoolSect->unlock();

	return ret;
}

void btParallelConstraintSolver::freeTask(void *ptr) {
	m_pTaskPoolSect->lock();
	if (m_pTaskPool->validPtr(ptr)) {
		m_pTaskPool->freeMemory(ptr);
	} else {
		btFree(ptr);
	}
	m_pTaskPoolSect->unlock();
}

btScalar btParallelConstraintSolver::solveGroup(btCollisionObject **bodies, int numBodies, btPersistentManifold **manifolds, int numManifolds, btTypedConstraint **constraints, 
											int numConstraints, const btContactSolverInfo &info, btIDebugDraw *debugDrawer, btStackAlloc *stackAlloc, btDispatcher *dispatcher) {


	// Unused return value
	return btScalar(0);
}

void btParallelConstraintSolver::waitUntilFinished() {
	m_pThreadPool->runTasks();
	m_pThreadPool->waitIdle();
}