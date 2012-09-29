#include "StdAfx.h"

#include "CPhysics.h"
#include "CPhysicsEnvironment.h"
#include "CPhysicsObjectPairHash.h"
#include "CPhysicsCollisionSet.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

//IPhysics* g_ValvePhysics = NULL;

void *CPhysics::QueryInterface(const char *pInterfaceName) {
	CreateInterfaceFn func = Sys_GetFactoryThis();
	return func(pInterfaceName, 0);
}

InitReturnVal_t CPhysics::Init() {
	InitReturnVal_t nRetVal = BaseClass::Init();
	if (nRetVal != INIT_OK) return nRetVal;
	/*m_valvephysics = Sys_LoadModule("vphysics_valve");
	CreateInterfaceFn factory = Sys_GetFactory(m_valvephysics);
	if (!factory) return INIT_FAILED;
	g_ValvePhysics = (IPhysics*)factory(VPHYSICS_INTERFACE_VERSION, NULL);
	g_ValvePhysicsCollision = (IPhysicsCollision*)factory(VPHYSICS_COLLISION_INTERFACE_VERSION, NULL);
	if (!g_ValvePhysicsCollision) return INIT_FAILED;*/
	return INIT_OK;
}

void CPhysics::Shutdown() {
	Sys_UnloadModule(m_valvephysics);
	m_valvephysics = NULL;
	BaseClass::Shutdown();
}

IPhysicsEnvironment* CPhysics::CreateEnvironment() {
	IPhysicsEnvironment *pEnvironment = new CPhysicsEnvironment;
	m_envList.AddToTail(pEnvironment);
	return pEnvironment;
}

void CPhysics::DestroyEnvironment(IPhysicsEnvironment *pEnvironment) {
	m_envList.FindAndRemove(pEnvironment);
	delete pEnvironment;
}

IPhysicsEnvironment* CPhysics::GetActiveEnvironmentByIndex(int index) {
	if (index < 0 || index >= m_envList.Count()) return NULL;
	return m_envList[index];
}

IPhysicsObjectPairHash* CPhysics::CreateObjectPairHash() {
	return new CPhysicsObjectPairHash();
}

void CPhysics::DestroyObjectPairHash(IPhysicsObjectPairHash *pHash) {
	delete (CPhysicsObjectPairHash*)pHash;
}

IPhysicsCollisionSet* CPhysics::FindOrCreateCollisionSet(unsigned int id, int maxElementCount) {
	if (m_collisionSets.IsValidIndex(id))
		return m_collisionSets[id];
	CPhysicsCollisionSet *set = new CPhysicsCollisionSet(maxElementCount);
	m_collisionSets.InsertBefore(id, set);
	return set;
}

IPhysicsCollisionSet* CPhysics::FindCollisionSet(unsigned int id) {
	if (m_collisionSets.IsValidIndex(id))
		return m_collisionSets[id];

	return NULL;
}

void CPhysics::DestroyAllCollisionSets() {
	for (int i = 0; i < m_collisionSets.Count(); i++)
		delete (CPhysicsCollisionSet*)m_collisionSets[i];

	m_collisionSets.RemoveAll();
}

static CPhysics g_MainDLLInterface;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysics, IPhysics, VPHYSICS_INTERFACE_VERSION, g_MainDLLInterface);
