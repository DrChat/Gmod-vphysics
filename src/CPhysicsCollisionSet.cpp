#include "StdAfx.h"

#include "CPhysicsCollisionSet.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/******************************
* CLASS CPhysicsCollisionSet
******************************/

CPhysicsCollisionSet::CPhysicsCollisionSet(int iMaxEntries) {
	m_iMaxEntries = iMaxEntries;
}

CPhysicsCollisionSet::~CPhysicsCollisionSet() {

}

void CPhysicsCollisionSet::EnableCollisions(int index0, int index1) {
	NOT_IMPLEMENTED;
}

void CPhysicsCollisionSet::DisableCollisions(int index0, int index1) {
	NOT_IMPLEMENTED;
}

bool CPhysicsCollisionSet::ShouldCollide(int index0, int index1) {
	NOT_IMPLEMENTED;
	return true;
}
