#ifndef CPHYSICSFLUIDCONTROLLER_H
#define CPHYSICSFLUIDCONTROLLER_H

class CPhysicsObject;

class CPhysicsFluidController : public IPhysicsFluidController
{
public:
	CPhysicsFluidController(CPhysicsObject *pFluidObject, fluidparams_t * pParams);
	virtual ~CPhysicsFluidController( void );

	virtual void	SetGameData( void *pGameData ) ;
	virtual void	*GetGameData( void ) const;

	virtual void	GetSurfacePlane( Vector *pNormal, float *pDist ) const;
	virtual float	GetDensity() const;
	virtual void	WakeAllSleepingObjects();
	virtual int		GetContents() const;
public:
	// Extended functions
private:
	void * m_pGameData;
	int m_iContents;
	float m_fDensity;
	Vector4D m_vSurfacePlane;
};

CPhysicsFluidController * CreateFluidController(CPhysicsObject *pFluidObject, fluidparams_t *pParams );

#endif