#include "StdAfx.h"

#include "Physics.h"
#include "Physics_Environment.h"
#include "Physics_ObjectPairHash.h"
#include "Physics_CollisionSet.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/******************
* CLASS CPhysics
******************/

CPhysics::~CPhysics() {
#if defined(_DEBUG) && defined(_MSC_VER)
	// Probably not the place we should be doing this, but who cares.
	_CrtDumpMemoryLeaks();
#endif
}

InitReturnVal_t CPhysics::Init() {
	InitReturnVal_t nRetVal = BaseClass::Init();
	if (nRetVal != INIT_OK) return nRetVal;

	return INIT_OK;
}

void CPhysics::Shutdown() {
	BaseClass::Shutdown();
}

void *CPhysics::QueryInterface(const char *pInterfaceName) {
	CreateInterfaceFn func = Sys_GetFactoryThis();
	if (!func)
		return NULL;

	return func(pInterfaceName, NULL);
}

IPhysicsEnvironment *CPhysics::CreateEnvironment() {
	IPhysicsEnvironment *pEnvironment = new CPhysicsEnvironment;
	m_envList.AddToTail(pEnvironment);
	return pEnvironment;
}

void CPhysics::DestroyEnvironment(IPhysicsEnvironment *pEnvironment) {
	m_envList.FindAndRemove(pEnvironment);
	delete (CPhysicsEnvironment *)pEnvironment;
}

IPhysicsEnvironment *CPhysics::GetActiveEnvironmentByIndex(int index) {
	if (index < 0 || index >= m_envList.Count()) return NULL;
	return m_envList[index];
}

// TODO: EXPOSE
int CPhysics::GetNumActiveEnvironments() {
	return m_envList.Count();
}

IPhysicsObjectPairHash *CPhysics::CreateObjectPairHash() {
	return new CPhysicsObjectPairHash();
}

void CPhysics::DestroyObjectPairHash(IPhysicsObjectPairHash *pHash) {
	delete (CPhysicsObjectPairHash *)pHash;
}

IPhysicsCollisionSet *CPhysics::FindOrCreateCollisionSet(unsigned int id, int maxElementCount) {
	if (m_collisionSets.IsValidIndex(id))
		return m_collisionSets[id];

	CPhysicsCollisionSet *set = ::CreateCollisionSet(maxElementCount);
	//m_collisionSets.InsertBefore(id, set); // FIXME: Assert hit with ragdolls!
	m_collisionSets.AddToTail(set);
	return set;
}

IPhysicsCollisionSet *CPhysics::FindCollisionSet(unsigned int id) {
	if (m_collisionSets.IsValidIndex(id))
		return m_collisionSets[id];

	return NULL;
}

void CPhysics::DestroyAllCollisionSets() {
	for (int i = 0; i < m_collisionSets.Count(); i++)
		delete (CPhysicsCollisionSet *)m_collisionSets[i];

	m_collisionSets.RemoveAll();
}

CPhysics g_Physics;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysics, IPhysics, VPHYSICS_INTERFACE_VERSION, g_Physics);
