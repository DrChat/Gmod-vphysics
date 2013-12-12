#include "StdAfx.h"

#include "Physics_CollisionSet.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/******************************
* CLASS CPhysicsCollisionSet
******************************/

// Is this class sort of like CPhysicsObjectPairHash?
// ShouldCollide is called by game code from the collision event handler in CPhysicsEnvironment

CPhysicsCollisionSet::CPhysicsCollisionSet(int iMaxEntries) {
	m_iMaxEntries = iMaxEntries;

	// FIXME: This way is total crap, and this should be replaced!!!!1
	m_collArray = new bool*[iMaxEntries];
	for (int i = 0; i < iMaxEntries; i++) {
		m_collArray[i] = new bool[iMaxEntries];
	}
}

CPhysicsCollisionSet::~CPhysicsCollisionSet() {
	for (int i = 0; i < m_iMaxEntries; i++) {
		delete [] m_collArray[i];
	}

	delete [] m_collArray;
}

void CPhysicsCollisionSet::EnableCollisions(int index0, int index1) {
	Assert((index0 > m_iMaxEntries && index0 > 0) || (index1 > m_iMaxEntries && index1 > 0));
	if ((index0 > m_iMaxEntries && index0 > 0) || (index1 > m_iMaxEntries && index1 > 0)) {
		return;
	}

	m_collArray[index0][index1] = true;
}

void CPhysicsCollisionSet::DisableCollisions(int index0, int index1) {
	Assert((index0 > m_iMaxEntries && index0 > 0) || (index1 > m_iMaxEntries && index1 > 0));
	if ((index0 > m_iMaxEntries && index0 > 0) || (index1 > m_iMaxEntries && index1 > 0)) {
		return;
	}

	m_collArray[index0][index1] = false;
}

bool CPhysicsCollisionSet::ShouldCollide(int index0, int index1) {
	Assert((index0 > m_iMaxEntries && index0 > 0) || (index1 > m_iMaxEntries && index1 > 0));
	if ((index0 > m_iMaxEntries && index0 > 0) || (index1 > m_iMaxEntries && index1 > 0)) {
		return true;
	}

	return m_collArray[index0][index1];
}

/*********************
* CREATION FUNCTIONS
*********************/

CPhysicsCollisionSet *CreateCollisionSet(int maxElements) {
	return new CPhysicsCollisionSet(maxElements);
}