#include "StdAfx.h"

#include "convert.h"
#include "Physics_FluidController.h"
#include "Physics_Object.h"
#include "Physics_Environment.h"
#include "Physics_SurfaceProps.h"
#include "Physics_Collision.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static void CalculateConvexBuoyancyForce(btConvexShape *pConvex, btTransform &objRelTrans, Vector4D fluidSurface, float fluidDensity, float convexBuoyancyRatio, btVector3 &force, btVector3 &torque) {
	static const btVector3 parallelDirections[4] = {
		btVector3( 1, 0, 0),
		btVector3(-1, 0, 0),
		btVector3( 0, 0, 1),
		btVector3( 0, 0,-1),
	};
	static const btVector3 up(0, 1, 0);

	btVector3 norm;
	ConvertDirectionToBull(fluidSurface.AsVector3D(), norm);
	for (int i = 0; i < 4; i++) {

	}
}

static void CalculateCompoundBuoyancyForce(btCompoundShape *pCompound, Vector4D fluidSurface, btScalar fluidDensity, btScalar ratio, btVector3 &force, btVector3 &torque) {
	
	
	for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
		
	}
}

// TODO: Buoyancy calculation
static void CalculateBuoyancyForce(CPhysicsObject *pObject, Vector4D fluidSurface, float fluidDensity) {
	btRigidBody *pBody = pObject->GetObject();
	btCollisionShape *pShape = pBody->getCollisionShape();

	if (pShape->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pShape;
		btVector3 force, torque;
		CalculateCompoundBuoyancyForce(pCompound, fluidSurface, fluidDensity, pObject->GetBuoyancyRatio(), force, torque);

		pBody->applyCentralForce(force);
		pBody->applyTorque(torque);
	} else if (pShape->isConvex()) {
		btConvexShape *pConvex = (btConvexShape *)pShape;
		btVector3 force, torque;
		CalculateConvexBuoyancyForce(pConvex, pBody->getWorldTransform(), fluidSurface, fluidDensity, pObject->GetBuoyancyRatio(), force, torque);

		pBody->applyCentralForce(force);
		pBody->applyTorque(torque);
	}
}

/********************************
* CLASS CPhysicsFluidCallback
********************************/

class CPhysicsFluidCallback : public btGhostObjectCallback {
	public:
		CPhysicsFluidCallback(CPhysicsFluidController *pController) {
			m_pController = pController;
		}

		void addedOverlappingObject(btCollisionObject *pObject) {
			CPhysicsObject *pPhys = (CPhysicsObject *)pObject->getUserPointer();
			if (!pPhys) return;

			m_pController->ObjectAdded(pPhys);
		}

		void removedOverlappingObject(btCollisionObject *pObject) {
			CPhysicsObject *pPhys = (CPhysicsObject *)pObject->getUserPointer();
			if (!pPhys) return;

			m_pController->ObjectRemoved(pPhys);
		}

	private:
		CPhysicsFluidController *m_pController;
};

/********************************
* CLASS CPhysicsFluidController
********************************/

CPhysicsFluidController::CPhysicsFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	m_pEnv = pEnv;
	m_pGameData = NULL;
	m_iContents = 0;
	m_vSurfacePlane = Vector4D(0, 0, 0, 0);

	if (pParams) {
		m_pGameData = pParams->pGameData;
		m_iContents = pParams->contents;
		m_vSurfacePlane = pParams->surfacePlane;
		ConvertPosToBull(pParams->currentVelocity, m_currentVelocity);
	}

	int matIndex = pFluidObject->GetMaterialIndex();
	surfacedata_t *pSurface = g_SurfaceDatabase.GetSurfaceData(matIndex);
	if (pSurface) {
		m_fDensity = pSurface->physics.density;
	}

	pFluidObject->EnableCollisions(false);
	pFluidObject->SetContents(m_iContents);
	pFluidObject->SetFluidController(this);

	m_pCallback = new CPhysicsFluidCallback(this);

	m_pGhostObject = new btGhostObject;
	m_pGhostObject->setUserPointer(pFluidObject);
	m_pGhostObject->setCallback(m_pCallback);
	m_pGhostObject->setCollisionShape(pFluidObject->GetObject()->getCollisionShape());
	m_pGhostObject->setWorldTransform(pFluidObject->GetObject()->getWorldTransform());
	m_pGhostObject->setCollisionFlags(m_pGhostObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE | btCollisionObject::CF_STATIC_OBJECT | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
	m_pEnv->GetBulletEnvironment()->addCollisionObject(m_pGhostObject, COLGROUP_WORLD, ~COLGROUP_WORLD);
}

CPhysicsFluidController::~CPhysicsFluidController() {
	m_pEnv->GetBulletEnvironment()->removeCollisionObject(m_pGhostObject);
	delete m_pGhostObject;
	delete m_pCallback;
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
	int numObjects = m_pGhostObject->getNumOverlappingObjects();
	for (int i = 0; i < numObjects; i++) {
		btRigidBody *body = btRigidBody::upcast(m_pGhostObject->getOverlappingObject(i));
		Assert(body);
		if (!body) continue;

		CPhysicsObject *pObject = (CPhysicsObject *)body->getUserPointer();
		Assert(pObject);

		btVector3 mins, maxs, omins, omaxs;
		body->getAabb(mins, maxs);
		m_pGhostObject->getCollisionShape()->getAabb(m_pGhostObject->getWorldTransform(), omins, omaxs);

		float height = maxs.y() - mins.y(); // If the plane for the surface can be non-upwards I'm going to murder something
		//float height = abs(ConvertDistanceToBull(m_vSurfacePlane.w));
		float dist = omaxs.y() - mins.y(); // Distance between top of water and bottom of object (how much of object is in water)
		float p = clamp(dist / height, 0.0f, 1.0f);
		float vol = (pObject->GetVolume() * p); // / 64; // Submerged volume

		// TODO: We need to calculate this force at several points on the object (How do we determine what points?).
		// Maybe have the points determined by the extents of the shape at 4 directions parallel to our surface
		// Simulate buoyant force per convex or just on the compound?

		// Simulate buoyant force per point submerged
		// Divide it over all of the points that are submerged

		// IVP calculates this force per triangle
		btVector3 force = (m_fDensity * -body->getGravity() * vol) * pObject->GetBuoyancyRatio();
		body->applyCentralForce(force);

		// Damping
		// FIXME: Damping would be way too much for an object only partially touching our surface (ex. giant fucking bridge with like 1 pixel in the water)
		body->setLinearVelocity(body->getLinearVelocity() * (1.0f - (0.75f * dt)));
		body->setAngularVelocity(body->getAngularVelocity() * (1.0f - (0.75f * dt)));
	}
}

// UNEXPOSED
void CPhysicsFluidController::ObjectAdded(CPhysicsObject *pObject) {
	m_pEnv->HandleFluidStartTouch(this, pObject);
}

// UNEXPOSED
void CPhysicsFluidController::ObjectRemoved(CPhysicsObject *pObject) {
	// Don't send the callback on objects that are being removed
	if (!pObject->IsBeingRemoved())
		m_pEnv->HandleFluidEndTouch(this, pObject);
}

void CPhysicsFluidController::TransferToEnvironment(CPhysicsEnvironment *pDest) {

}

/************************
* CREATION FUNCTIONS
************************/

CPhysicsFluidController *CreateFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	if (!pEnv || !pFluidObject) return NULL;
	CPhysicsFluidController *pFluid = new CPhysicsFluidController(pEnv, pFluidObject, pParams);
	return pFluid;
}
