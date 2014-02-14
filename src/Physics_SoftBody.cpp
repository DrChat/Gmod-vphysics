#include "StdAfx.h"

#include "convert.h"
#include "Physics_SoftBody.h"
#include "Physics_Environment.h"
#include "Physics_Object.h"

#include "BulletSoftBody/btSoftBodyHelpers.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

void ConvertNodeToHL(const btSoftBody::Node *node, softbodynode_t &nodeOut) {
	ConvertPosToHL(node->m_x, nodeOut.pos);
	ConvertPosToHL(node->m_v, nodeOut.vel);
	nodeOut.invMass = node->m_im;
}

void ConvertNodeToBull(const softbodynode_t &node, btSoftBody::Node &nodeOut) {
	ConvertPosToBull(node.pos, nodeOut.m_x);
	ConvertPosToBull(node.vel, nodeOut.m_v);
	nodeOut.m_im = node.invMass;
}

void ConvertFaceToHL(const btSoftBody::Face *face, softbodyface_t &faceOut) {
	for (int i = 0; i < 3; i++) {
		btSoftBody::Node *node = face->m_n[i];
		ConvertNodeToHL(node, faceOut.nodes[i]);
	}

	ConvertDirectionToHL(face->m_normal, faceOut.normal);
}

void ConvertLinkToHL(const btSoftBody::Link *link, softbodylink_t &linkOut) {
	for (int i = 0; i < 2; i++) {
		btSoftBody::Node *node = link->m_n[i];
		ConvertNodeToHL(node, linkOut.nodes[i]);
	}
}

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

void CPhysicsSoftBody::SetTotalMass(float fMass, bool bFromFaces) {
	m_pSoftBody->setTotalMass(fMass, bFromFaces);
}

void CPhysicsSoftBody::Anchor(int node, IPhysicsObject *pObj) {
	m_pSoftBody->appendAnchor(node, ((CPhysicsObject *)pObj)->GetObject());
}

int CPhysicsSoftBody::GetNodeCount() const {
	return m_pSoftBody->m_nodes.size();
}

int CPhysicsSoftBody::GetFaceCount() const {
	return m_pSoftBody->m_faces.size();
}

int CPhysicsSoftBody::GetLinkCount() const {
	return m_pSoftBody->m_links.size();
}

softbodynode_t CPhysicsSoftBody::GetNode(int i) const {
	btSoftBody::Node &node = m_pSoftBody->m_nodes[i];

	softbodynode_t out;
	ConvertNodeToHL(&node, out);
	return out;
}

softbodyface_t CPhysicsSoftBody::GetFace(int i) const {
	btSoftBody::Face &face = m_pSoftBody->m_faces[i];

	softbodyface_t out;
	ConvertFaceToHL(&face, out);
	return out;
}

softbodylink_t CPhysicsSoftBody::GetLink(int i) const {
	btSoftBody::Link &link = m_pSoftBody->m_links[i];

	softbodylink_t out;
	ConvertLinkToHL(&link, out);
	return out;
}

void CPhysicsSoftBody::SetNode(int i, softbodynode_t &node) {
	Assert(i >= 0 && i < m_pSoftBody->m_nodes.size());

	btSoftBody::Node &bnode = m_pSoftBody->m_nodes[i];
	ConvertNodeToBull(node, bnode);
}

void CPhysicsSoftBody::GetAABB(Vector *mins, Vector *maxs) const {
	if (!mins && !maxs) return;

	btVector3 btmins, btmaxs;
	m_pSoftBody->getAabb(btmins, btmaxs);

	Vector tMins, tMaxs;
	ConvertAABBToHL(btmins, btmaxs, tMins, tMaxs);

	if (mins)
		*mins = tMins;

	if (maxs)
		*maxs = tMaxs;
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

CPhysicsSoftBody *CreateSoftBodyFromTriMesh(CPhysicsEnvironment *pEnv, const Vector *vertices, int numVertices, const int *indices, int numIndices, const Vector &position, const QAngle &angles, const softbodyparams_t *pParams) {
	/*
	btVector3 *bullVerts = new btVector3[numVertices];

	// Make sure numIndices is evenly divisible by 3
	Assert(numIndices % 3 == 0);

	for (int i = 0; i < numVertices; i++) {
		ConvertPosToBull(vertices[i], bullVerts[i]);
	}

	delete [] bullVerts;
	*/

	NOT_IMPLEMENTED
	return NULL;
}

CPhysicsSoftBody *CreateSoftBodyFromVertices(CPhysicsEnvironment *pEnv, const Vector *vertices, int numVertices, const Vector &position, const QAngle &angles, const softbodyparams_t *pParams) {
	btVector3 *bullVerts = new btVector3[numVertices];

	for (int i = 0; i < numVertices; i++) {
		ConvertPosToBull(vertices[i], bullVerts[i]);
	}

	btSoftBodyWorldInfo wi = pEnv->GetSoftBodyWorldInfo();

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

CPhysicsSoftBody *CreateSoftBodyRope(CPhysicsEnvironment *pEnv, const Vector &position, const Vector &length, const softbodyparams_t *pParams) {
	btSoftBodyWorldInfo &wi = pEnv->GetSoftBodyWorldInfo();

	btVector3 start, end;
	ConvertPosToBull(position, start);
	ConvertPosToBull(position + length, end);

	// FIXME: Figure out what last 2 parameters are
	// last parameter is fixed side
	btSoftBody *pSoftBody = btSoftBodyHelpers::CreateRope(wi, start, end, 8, 0);

	CPhysicsSoftBody *pPhysBody = new CPhysicsSoftBody;
	pPhysBody->Init(pEnv, pSoftBody);

	return pPhysBody;
}