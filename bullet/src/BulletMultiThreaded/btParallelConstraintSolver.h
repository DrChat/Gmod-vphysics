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

#ifndef __BT_PARALLEL_CONSTRAINT_SOLVER_H
#define __BT_PARALLEL_CONSTRAINT_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "LinearMath/btScalar.h"
#include "PlatformDefinitions.h"

class btThreadPool;

/// The btParallelConstraintSolver performs computations on constraint rows in parallel
class btParallelConstraintSolver : public btSequentialImpulseConstraintSolver {
	public:
		btParallelConstraintSolver(btThreadPool *pThreadPool);
		virtual ~btParallelConstraintSolver();
	
		virtual btScalar solveGroup(btCollisionObject **bodies, int numBodies, btPersistentManifold **manifold, int numManifolds, btTypedConstraint **constraints, int numConstraints, const btContactSolverInfo &info, btIDebugDraw *debugDrawer, btStackAlloc *stackAlloc, btDispatcher *dispatcher);

	protected:
		btThreadPool *m_pThreadPool;
};



#endif //__BT_PARALLEL_CONSTRAINT_SOLVER_H