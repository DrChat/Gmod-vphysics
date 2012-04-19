#ifndef CPHYSICSFLUIDCONTROLLER_H
#define CPHYSICSFLUIDCONTROLLER_H

class CPhysicsObject;

class CPhysicsFluidController : public IPhysicsFluidController
{
public:
	CPhysicsFluidController(CPhysicsObject *pFluidObject);
	virtual ~CPhysicsFluidController( void );

	virtual void	SetGameData( void *pGameData ) ;
	virtual void	*GetGameData( void ) const;

	virtual void	GetSurfacePlane( Vector *pNormal, float *pDist ) const;
	virtual float	GetDensity() const;
	virtual void	WakeAllSleepingObjects();
	virtual int		GetContents() const;
private:
	void * m_pGameData;
};

CPhysicsFluidController * CreateFluidController(CPhysicsObject *pFluidObject, fluidparams_t *pParams );

#endif