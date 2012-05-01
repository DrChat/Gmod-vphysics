#include "StdAfx.h"

#include "CPhysicsCollisionSet.h"

CPhysicsCollisionSet::CPhysicsCollisionSet(int iMaxEntries)
{
	m_iMaxEntries = iMaxEntries;
}

CPhysicsCollisionSet::~CPhysicsCollisionSet()
{
}

void CPhysicsCollisionSet::EnableCollisions(int index0, int index1)
{
	NOT_IMPLEMENTED;
}

void CPhysicsCollisionSet::DisableCollisions(int index0, int index1)
{
	NOT_IMPLEMENTED;
}

bool CPhysicsCollisionSet::ShouldCollide(int index0, int index1)
{
	NOT_IMPLEMENTED;
	return true;
}
