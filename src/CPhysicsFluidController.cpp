#include "StdAfx.h"

#include "CPhysicsFluidController.h"
#include "CPhysicsObject.h"
#include "CPhysicsEnvironment.h"

#include "tier0/vprof.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

CPhysicsFluidController *CreateFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	if (!pEnv || !pFluidObject) return NULL;
	CPhysicsFluidController *pFluid = new CPhysicsFluidController(pEnv, pFluidObject, pParams);
	return pFluid;
}

/********************************
* CLASS CPhysicsFluidController
********************************/

CPhysicsFluidController::CPhysicsFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	m_pEnv = pEnv;
	m_pGameData = NULL;
	m_iContents = 0;
	m_vSurfacePlane = Vector4D(0, 0, 0, 0);

	Assert(pEnv);
	Assert(pFluidObject);

	if (pParams) {
		m_pGameData = pParams->pGameData;
		m_iContents = pParams->contents;
		m_vSurfacePlane = pParams->surfacePlane;
	}

	m_fDensity = 1.0f;	// Density of water, used to be a parameter for this in the 2003 leak but it seems to have been removed for some reason
						// FIXME: This might be available in the surface properities of the fluid object?
	pFluidObject->EnableCollisions(false);
	pFluidObject->SetContents(m_iContents);
	pFluidObject->SetFluidController(this);

	m_pGhostObject = new btGhostObject;
	m_pGhostObject->setUserPointer(pFluidObject);
	m_pGhostObject->setCollisionShape(pFluidObject->GetObject()->getCollisionShape());
	m_pGhostObject->setWorldTransform(pFluidObject->GetObject()->getWorldTransform());
	m_pGhostObject->setCollisionFlags(m_pGhostObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE | btCollisionObject::CF_STATIC_OBJECT);
	m_pEnv->GetBulletEnvironment()->addCollisionObject(m_pGhostObject, 2, ~2);
}

CPhysicsFluidController::~CPhysicsFluidController() {
	m_pEnv->GetBulletEnvironment()->removeCollisionObject(m_pGhostObject);
	delete m_pGhostObject;
}

void CPhysicsFluidController::WakeAllSleepingObjects() {
	int count = m_pGhostObject->getNumOverlappingObjects();
	for (int i = 0; i < count; i++) {
		btRigidBody *body = btRigidBody::upcast(m_pGhostObject->getOverlappingObject(i));
		if (!body)
			continue;

		body->activate(true);
	}
}

void CPhysicsFluidController::SetGameData(void *pGameData) {
	m_pGameData = pGameData;
}

void *CPhysicsFluidController::GetGameData() const {
	return m_pGameData;
}

void CPhysicsFluidController::GetSurfacePlane(Vector *pNormal, float *pDist) const {
	if (!pNormal && !pDist) return;

	if (pNormal)
		*pNormal = m_vSurfacePlane.AsVector3D();

	if (pDist)
		*pDist = m_vSurfacePlane.w;
}

float CPhysicsFluidController::GetDensity() const {
	return m_fDensity;
}

int	CPhysicsFluidController::GetContents() const {
	return m_iContents;
}

// TODO: Refactor this code to be less messy.
void CPhysicsFluidController::Tick(float dt) {
	VPROF_BUDGET("CPhysicsFluidController::Tick", VPROF_BUDGETGROUP_PHYSICS);

	int numObjects = m_pGhostObject->getNumOverlappingObjects();
	for (int i = 0; i < numObjects; i++) {
		btRigidBody *body = btRigidBody::upcast(m_pGhostObject->getOverlappingObject(i));
		if (!body)
			continue;

		CPhysicsObject *pObject = (CPhysicsObject *)body->getUserPointer();
		if (!pObject)
			continue;

		btVector3 mins, maxs, omins, omaxs;
		body->getAabb(mins, maxs);
		m_pGhostObject->getCollisionShape()->getAabb(m_pGhostObject->getWorldTransform(), omins, omaxs);

		float height = maxs.y() - mins.y(); // If the plane for the surface can be non-upwards I'm going to murder something
		float dist = omaxs.y() - mins.y();
		float p = clamp(dist / height, 0.0f, 1.0f);
		float vol = (pObject->GetVolume() * p) / 64; 

		body->applyCentralForce((-body->getGravity() * m_fDensity * vol) * pObject->GetBuoyancyRatio()/*(maxs + mins - btVector3(0,height*(1-p),0)) * 0.5f - body->getWorldTransform().getOrigin()*/);

		// Damping
		body->setLinearVelocity(body->getLinearVelocity() * (1.0f - (0.75f * dt)));
		body->setAngularVelocity(body->getAngularVelocity() * (1.0f - (0.75f * dt)));

		if (body->getLinearVelocity().length2() > 0.01f)
			body->activate(true); // Stop it from freezing while mini-bouncing, it looks dumb
	}
}