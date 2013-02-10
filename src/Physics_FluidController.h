#ifndef CPHYSICSFLUIDCONTROLLER_H
#define CPHYSICSFLUIDCONTROLLER_H

class CPhysicsObject;
class CPhysicsEnvironment;

#include "IController.h"

class CPhysicsFluidController : public IPhysicsFluidController, public IController
{
	public:
								CPhysicsFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams);
								~CPhysicsFluidController();

		void					SetGameData(void *pGameData);
		void *					GetGameData() const;

		void					GetSurfacePlane(Vector *pNormal, float *pDist) const;
		float					GetDensity() const;
		void					WakeAllSleepingObjects();
		int						GetContents() const;
		void					Tick(float deltaTime);
	private:
		void *					m_pGameData;
		int						m_iContents;
		float					m_fDensity;
		Vector4D				m_vSurfacePlane;
		CPhysicsEnvironment *	m_pEnv;
		btGhostObject *			m_pGhostObject;
};

CPhysicsFluidController *CreateFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams);

#endif