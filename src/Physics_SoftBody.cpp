#include "StdAfx.h"

#include "convert.h"
#include "Physics_SoftBody.h"
#include "Physics_Environment.h"

#include "BulletSoftBody/btSoftBodyHelpers.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/*************************
* CLASS CPhysicsSoftBody
*************************/

CPhysicsSoftBody::CPhysicsSoftBody() {
	m_pEnv = NULL;
	m_pSoftBody = NULL;
}

CPhysicsSoftBody::~CPhysicsSoftBody() {
	m_pEnv->GetBulletEnvironment()->removeSoftBody(m_pSoftBody);
	delete m_pSoftBody;
}

void CPhysicsSoftBody::Init(CPhysicsEnvironment *pEnv, btSoftBody *pSoftBody) {
	m_pEnv			= pEnv;
	m_pSoftBody		= pSoftBody;

	pEnv->GetBulletEnvironment()->addSoftBody(m_pSoftBody);
}

btSoftBody *CPhysicsSoftBody::GetSoftBody() {
	return m_pSoftBody;
}

/*************************
* CREATION FUNCTIONS
*************************/

CPhysicsSoftBody *CreateSoftBodyFromTriMesh(CPhysicsEnvironment *pEnv) {
	NOT_IMPLEMENTED
	return NULL;
}

CPhysicsSoftBody *CreateSoftBodyFromVertices(CPhysicsEnvironment *pEnv, const Vector *vertices, int numVertices, const Vector &position, const QAngle &angles) {
	btVector3 *bullVerts = new btVector3[numVertices];

	for (int i = 0; i < numVertices; i++) {
		ConvertPosToBull(vertices[i], bullVerts[i]);
	}

	btSoftBodyWorldInfo wi;
	wi.air_density = pEnv->GetAirDensity();
	wi.m_broadphase = pEnv->GetBulletEnvironment()->getBroadphase();
	wi.m_dispatcher = pEnv->GetBulletEnvironment()->getDispatcher();
	wi.m_gravity = pEnv->GetBulletEnvironment()->getGravity();
	//wi.m_sparsesdf = 0;

	btSoftBody *pSoftBody = btSoftBodyHelpers::CreateFromConvexHull(wi, bullVerts, numVertices);
	delete [] bullVerts;

	btVector3 bullPos;
	btMatrix3x3 bullAng;
	ConvertPosToBull(position, bullPos);
	ConvertRotationToBull(angles, bullAng);

	pSoftBody->setWorldTransform(btTransform(bullAng, bullPos));

	CPhysicsSoftBody *pBody = new CPhysicsSoftBody;
	pBody->Init(pEnv, pSoftBody);

	return pBody;
}