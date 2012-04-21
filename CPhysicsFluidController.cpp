#include "StdAfx.h"

#include "CPhysicsFluidController.h"

CPhysicsFluidController * CreateFluidController(CPhysicsObject *pFluidObject, fluidparams_t *pParams )
{
	CPhysicsFluidController *pFluid = new CPhysicsFluidController( pFluidObject, pParams );
	pFluid->SetGameData( pParams->pGameData );
	return pFluid;
}

CPhysicsFluidController::CPhysicsFluidController(CPhysicsObject *pFluidObject, fluidparams_t * pParams)
{
	m_pGameData = NULL;
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
	NOT_IMPLEMENTED;
}
float	CPhysicsFluidController::GetDensity() const
{
	NOT_IMPLEMENTED;
	return 0;
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