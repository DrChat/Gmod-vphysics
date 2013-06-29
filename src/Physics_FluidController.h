#ifndef PHYSICS_FLUIDCONTROLLER_H
#define PHYSICS_FLUIDCONTROLLER_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

class CPhysicsObject;
class CPhysicsEnvironment;

class CPhysicsFluidCallback;

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

		// UNEXPOSED FUNCTIONS
	public:
		void					Tick(float deltaTime);

		void					ObjectAdded(CPhysicsObject *pObject);
		void					ObjectRemoved(CPhysicsObject *pObject);
	private:
		void *					m_pGameData;
		int						m_iContents;
		float					m_fDensity;

		// surface plane is a Vector surface normal and float distance
		Vector4D				m_vSurfacePlane;
		CPhysicsEnvironment *	m_pEnv;
		btGhostObject *			m_pGhostObject;
		CPhysicsFluidCallback *	m_pCallback;
};

CPhysicsFluidController *CreateFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams);

#endif // PHYSICS_FLUIDCONTROLLER_H
