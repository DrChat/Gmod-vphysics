#include "StdAfx.h"

#include "CPhysicsFluidController.h"
#include "CPhysicsObject.h"

CPhysicsFluidController * CreateFluidController(CPhysicsObject *pFluidObject, fluidparams_t *pParams )
{
	CPhysicsFluidController *pFluid = new CPhysicsFluidController( pFluidObject, pParams );
	pFluid->SetGameData( pParams->pGameData );
	return pFluid;
}

CPhysicsFluidController::CPhysicsFluidController(CPhysicsObject *pFluidObject, fluidparams_t * pParams)
{
	m_pGameData = pParams->pGameData;
	m_iContents = pParams->contents;
	m_fDensity = 1000.0f; // Density of water (1000 kg/m^3), used to be a parameter for this in the 2003 leak but it seems to have been removed for some reason
	m_vSurfacePlane = pParams->surfacePlane;
	pFluidObject->EnableCollisions(false);
	pFluidObject->SetContents(m_iContents); // Do we really need to do this?
	pFluidObject->SetFluidController(this);
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
	NOT_IMPLEMENTED;
	return 0;
}