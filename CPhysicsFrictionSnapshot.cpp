#include "StdAfx.h"

#include "CPhysicsFrictionSnapshot.h"

CPhysicsFrictionSnapshot::CPhysicsFrictionSnapshot() {

}

CPhysicsFrictionSnapshot::~CPhysicsFrictionSnapshot() {

}

bool CPhysicsFrictionSnapshot::IsValid() {
	NOT_IMPLEMENTED;
	return false;
}

IPhysicsObject* CPhysicsFrictionSnapshot::GetObject(int index) {
	NOT_IMPLEMENTED;
	return NULL;
}

int CPhysicsFrictionSnapshot::GetMaterial(int index) {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsFrictionSnapshot::GetContactPoint(Vector& out) {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::GetSurfaceNormal(Vector& out) {
	NOT_IMPLEMENTED;
}

float CPhysicsFrictionSnapshot::GetNormalForce() {
	NOT_IMPLEMENTED;
	return 0;
}

float CPhysicsFrictionSnapshot::GetEnergyAbsorbed() {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsFrictionSnapshot::RecomputeFriction() {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::ClearFrictionForce() {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::MarkContactForDelete() {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::DeleteAllMarkedContacts(bool wakeObjects) {
	NOT_IMPLEMENTED;
}

void CPhysicsFrictionSnapshot::NextFrictionData() {
	NOT_IMPLEMENTED;
}

float CPhysicsFrictionSnapshot::GetFrictionCoefficient() {
	NOT_IMPLEMENTED;
	return 0;
}

