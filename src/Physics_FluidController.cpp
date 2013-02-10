#include "StdAfx.h"

#include "Physics_FluidController.h"
#include "Physics_Object.h"
#include "Physics_Environment.h"
#include "Physics_SurfaceProps.h"

#include "tier0/vprof.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

extern CPhysicsSurfaceProps g_SurfaceDatabase;

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

	m_fDensity = 1.0f;

	// FIXME: The below density is correct (kg/m^3, close to 1000). Adjust our calculations in Tick
	/*
	int matIndex = pFluidObject->GetMaterialIndex();
	surfacedata_t *pSurface = g_SurfaceDatabase.GetSurfaceData(matIndex);
	if (pSurface) {
		m_fDensity = pSurface->physics.density;
	}
	*/

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

		// TODO: We need to calculate this force at several points on the object.
		btVector3 force = (m_fDensity * -body->getGravity() * vol) * pObject->GetBuoyancyRatio();
		body->applyCentralForce(force);

		/*
		int numManifolds = m_pEnv->GetBulletEnvironment()->getDispatcher()->getNumManifolds();
		for (int j = 0; j < numManifolds; j++) {
			btPersistentManifold *pManifold = m_pEnv->GetBulletEnvironment()->getDispatcher()->getManifoldByIndexInternal(j);
			const btCollisionObject *obA = pManifold->getBody0();
			const btCollisionObject *obB = pManifold->getBody1();

			int numContacts = pManifold->getNumContacts();
			if (numContacts <= 0)
				continue;

			if (obA == m_pGhostObject && obB == body) {
				for (int k = 0; k < numContacts; k++) {
					btManifoldPoint manPoint = pManifold->getContactPoint(k);
					btVector3 pos = manPoint.getPositionWorldOnB();

					body->applyForce(force, pos);
				}
			} else if (obB == m_pGhostObject && obA == body) {
				for (int k = 0; k < numContacts; k++) {
					btManifoldPoint manPoint = pManifold->getContactPoint(k);
					btVector3 pos = manPoint.getPositionWorldOnA();

					body->applyForce(force, pos);
				}
			}
		}
		*/

		// Damping
		// FIXME: Damping would be way too much for an object barely touching our surface.
		body->setLinearVelocity(body->getLinearVelocity() * (1.0f - (0.75f * dt)));
		body->setAngularVelocity(body->getAngularVelocity() * (1.0f - (0.75f * dt)));

		if (body->getLinearVelocity().length2() > 0.01f)
			body->activate(true); // Stop it from freezing while mini-bouncing, it looks dumb
	}
}

/************************
* CREATION FUNCTIONS
************************/

CPhysicsFluidController *CreateFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	if (!pEnv || !pFluidObject) return NULL;
	CPhysicsFluidController *pFluid = new CPhysicsFluidController(pEnv, pFluidObject, pParams);
	return pFluid;
}
