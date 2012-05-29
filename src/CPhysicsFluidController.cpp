#include "StdAfx.h"

#include "CPhysicsFluidController.h"
#include "CPhysicsObject.h"
#include "CPhysicsEnvironment.h"

CPhysicsFluidController * CreateFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams)
{
	CPhysicsFluidController *pFluid = new CPhysicsFluidController( pEnv, pFluidObject, pParams );
	pFluid->SetGameData( pParams->pGameData );
	return pFluid;
}

CPhysicsFluidController::CPhysicsFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t * pParams)
{
	m_pEnv = pEnv;
	m_pGameData = pParams->pGameData;
	m_iContents = pParams->contents;
	m_fDensity = 1.0f; // Density of water, used to be a parameter for this in the 2003 leak but it seems to have been removed for some reason
	m_vSurfacePlane = pParams->surfacePlane;
	pFluidObject->EnableCollisions(false);
	pFluidObject->SetContents(m_iContents); // Do we really need to do this?
	pFluidObject->SetFluidController(this);
	m_pGhostObject = new btGhostObject();
	m_pGhostObject->setCollisionShape(pFluidObject->GetObject()->getCollisionShape());
	m_pGhostObject->setWorldTransform(pFluidObject->GetObject()->getWorldTransform());
	m_pGhostObject->setCollisionFlags(m_pGhostObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE | btCollisionObject::CF_STATIC_OBJECT);
	m_pEnv->GetBulletEnvironment()->addCollisionObject(m_pGhostObject, 2, ~2);
}
CPhysicsFluidController::~CPhysicsFluidController( void ) 
{
}

void CPhysicsFluidController::SetGameData( void *pGameData ) 
{
	m_pGameData = pGameData;
}
void * CPhysicsFluidController::GetGameData( void ) const
{
	return m_pGameData;
}

void CPhysicsFluidController::GetSurfacePlane( Vector *pNormal, float *pDist ) const
{
	*pNormal = m_vSurfacePlane.AsVector3D();
	*pDist = m_vSurfacePlane.w;
}

float CPhysicsFluidController::GetDensity() const
{
	return m_fDensity;
}

void CPhysicsFluidController::WakeAllSleepingObjects()
{
	NOT_IMPLEMENTED;
}

int	CPhysicsFluidController::GetContents() const
{
	return m_iContents;
}

void CPhysicsFluidController::Tick(float dt)
{
	int count = m_pGhostObject->getNumOverlappingObjects();
	for (int i = 0; i < count; i++)
	{
		btRigidBody *body = btRigidBody::upcast(m_pGhostObject->getOverlappingObject(i));
		if (!body) continue;
		CPhysicsObject *obj = (CPhysicsObject*)body->getUserPointer();
		btVector3 mins, maxs, omins, omaxs;
		body->getAabb(mins, maxs);
		float height = maxs.y() - mins.y(); // If the plane for the surface can be non-upwards I'm going to murder something
		m_pGhostObject->getCollisionShape()->getAabb(m_pGhostObject->getWorldTransform(), omins, omaxs);
		float dist = omaxs.y() - mins.y();
		float p = clamp(dist/height, 0.0f, 1.0f);
		float vol = (obj->GetVolume() * p)/64; 
		body->applyCentralForce((-body->getGravity() * m_fDensity * vol)*obj->GetBuoyancyRatio()/*, (maxs + mins - btVector3(0,height*(1-p),0)) * 0.5f - body->getWorldTransform().getOrigin()*/);
		body->setLinearVelocity(body->getLinearVelocity() * (1.0f-(0.75f*dt)));
		body->setAngularVelocity(body->getAngularVelocity() * (1.0f-(0.75f*dt)));
		if (body->getLinearVelocity().length2() > 0.01f)
			body->activate(true); // Stop it from freezing while mini-bouncing, it looks dumb
	}
}