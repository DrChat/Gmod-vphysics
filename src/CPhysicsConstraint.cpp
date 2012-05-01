#include "StdAfx.h"

#include "CPhysicsObject.h"
#include "CPhysicsConstraint.h"
#include "CPhysicsEnvironment.h"

CPhysicsConstraint::CPhysicsConstraint(CPhysicsEnvironment *pEnv, CPhysicsObject* pObject1, CPhysicsObject* pObject2, btTypedConstraint *pConstraint)
{
	m_pObject1 = pObject1;
	m_pObject2 = pObject2;
	m_pConstraint = pConstraint;
	m_pEnv = pEnv;
}

CPhysicsConstraint::~CPhysicsConstraint()
{
	m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);
	delete m_pConstraint;
}

void CPhysicsConstraint::Activate(void)
{
	m_pConstraint->setEnabled(true);
}

void CPhysicsConstraint::Deactivate(void)
{
	m_pConstraint->setEnabled(false);
}