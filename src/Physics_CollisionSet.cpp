#include "StdAfx.h"

#include "Physics_CollisionSet.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/******************************
* CLASS CPhysicsCollisionSet
******************************/

// Is this class sort of like CPhysicsObjectPairHash?
// ShouldCollide is called by game code from the collision event handler in CPhysicsEnvironment

CPhysicsCollisionSet::CPhysicsCollisionSet(int iMaxEntries) {
	m_iMaxEntries = iMaxEntries;
}

CPhysicsCollisionSet::~CPhysicsCollisionSet() {

}

void CPhysicsCollisionSet::EnableCollisions(int index0, int index1) {
	NOT_IMPLEMENTED
}

void CPhysicsCollisionSet::DisableCollisions(int index0, int index1) {
	NOT_IMPLEMENTED
}

bool CPhysicsCollisionSet::ShouldCollide(int index0, int index1) {
	NOT_IMPLEMENTED
	return true;
}

/*********************
* CREATION FUNCTIONS
*********************/

CPhysicsCollisionSet *CreateCollisionSet(int maxElements) {
	return new CPhysicsCollisionSet(maxElements);
}