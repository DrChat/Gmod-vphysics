#include "StdAfx.h"

#include "CPhysicsCollision.h"
#include "convert.h"

IPhysicsCollision* g_ValvePhysicsCollision = NULL;

CPhysicsCollision::CPhysicsCollision() {

}

CPhysicsCollision::~CPhysicsCollision() {

}

CPhysConvex* CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	NOT_IMPLEMENTED;
	return NULL;
}

CPhysConvex* CPhysicsCollision::ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance) {
	NOT_IMPLEMENTED;
	return NULL;
}

float CPhysicsCollision::ConvexVolume(CPhysConvex *pConvex) {
	NOT_IMPLEMENTED;
	return 0;
}

float CPhysicsCollision::ConvexSurfaceArea(CPhysConvex *pConvex) {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsCollision::SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData) {
	NOT_IMPLEMENTED;
}

void CPhysicsCollision::ConvexFree(CPhysConvex *pConvex) {
	NOT_IMPLEMENTED;
}

CPhysConvex* CPhysicsCollision::BBoxToConvex(const Vector &mins, const Vector &maxs) {
	NOT_IMPLEMENTED;
	return NULL;
}
CPhysConvex* CPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsCollision::ConvexesFromConvexPolygon(const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput) {
	NOT_IMPLEMENTED;
}

CPhysPolysoup* CPhysicsCollision::PolysoupCreate() {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsCollision::PolysoupDestroy(CPhysPolysoup *pSoup) {
	NOT_IMPLEMENTED;
}

void CPhysicsCollision::PolysoupAddTriangle(CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits) {
	NOT_IMPLEMENTED;
}

CPhysCollide* CPhysicsCollision::ConvertPolysoupToCollide(CPhysPolysoup *pSoup, bool useMOPP) {
	NOT_IMPLEMENTED;
	return NULL;
}

CPhysCollide* CPhysicsCollision::ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount) {
	NOT_IMPLEMENTED;
	return NULL;
}

CPhysCollide* CPhysicsCollision::ConvertConvexToCollideParams(CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsCollision::DestroyCollide(CPhysCollide *pCollide) {
	btCollisionShape* shape = (btCollisionShape*)pCollide;
	delete shape;
}

int CPhysicsCollision::CollideSize(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED;
	return 0;
}

int CPhysicsCollision::CollideWrite(char *pDest, CPhysCollide *pCollide, bool bSwap) {
	NOT_IMPLEMENTED;
	return 0;
}

CPhysCollide* CPhysicsCollision::UnserializeCollide(char *pBuffer, int size, int index) {
	NOT_IMPLEMENTED;
	return NULL;
}

float CPhysicsCollision::CollideVolume(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED;
	return 0;
}

float CPhysicsCollision::CollideSurfaceArea(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED;
	return 0;
}

Vector CPhysicsCollision::CollideGetExtent(const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction) {
	NOT_IMPLEMENTED;
	return Vector();
}

void CPhysicsCollision::CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles) {
	// Bullet returns very different AABBs than Havok.
	btCollisionShape* shape = (btCollisionShape*)pCollide;

	btVector3 pos, mins, maxs;
	btMatrix3x3 rot;

	ConvertPosToBull(collideOrigin, pos);
	ConvertRotationToBull(collideAngles, rot);
	btTransform transform(rot, pos);

	shape->getAabb(transform, mins, maxs);

	PhysicsShapeInfo *shapeInfo = (PhysicsShapeInfo*)shape->getUserPointer();
	if (shapeInfo)
	{
		// FIXME: correct the AABB's after the mass center here
	}

	ConvertPosToHL(mins, *pMins);
	ConvertPosToHL(maxs, *pMaxs);
}

void CPhysicsCollision::CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter) {
	NOT_IMPLEMENTED;
}

void CPhysicsCollision::CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter) {
	NOT_IMPLEMENTED;
}

Vector CPhysicsCollision::CollideGetOrthographicAreas(const CPhysCollide *pCollide) {
	NOT_IMPLEMENTED;
	return Vector();
}

void CPhysicsCollision::CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas) {
	NOT_IMPLEMENTED;
}

int CPhysicsCollision::CollideIndex(const CPhysCollide *pCollide) {
	NOT_IMPLEMENTED;
	return 0;
}

CPhysCollide* CPhysicsCollision::BBoxToCollide(const Vector &mins, const Vector &maxs) {
	if (mins == maxs) return NULL;

	btVector3 btmins, btmaxs;
	ConvertPosToBull(mins, btmins);
	ConvertPosToBull(maxs, btmaxs);
	btVector3 halfsize = (btmaxs - btmins)/2;

	btBoxShape* box = new btBoxShape(halfsize);
	btCompoundShape* shape = new btCompoundShape;

	btTransform transform(btMatrix3x3::getIdentity(), btmins + halfsize);

	shape->addChildShape(transform, box);

	return (CPhysCollide*)shape;
}

int CPhysicsCollision::GetConvexesUsedInCollideable(const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit) {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsCollision::TraceBox(const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	NOT_IMPLEMENTED;
}

void CPhysicsCollision::TraceBox(const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	NOT_IMPLEMENTED;
}

void CPhysicsCollision::TraceBox(const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	btVector3 btvec;
	btMatrix3x3 btmatrix;
	btCollisionObject* object = new btCollisionObject;
	btCollisionShape* shape = (btCollisionShape*)pCollide;
	object->setCollisionShape(shape);
	ConvertPosToBull(collideOrigin, btvec);
	ConvertRotationToBull(collideAngles, btmatrix);
	btTransform transform(btmatrix, btvec);
	object->setWorldTransform(transform);

	btVector3 startv, endv;
	ConvertPosToBull(ray.m_Start, startv);
	ConvertPosToBull(ray.m_Start + ray.m_Delta, endv);
	btTransform startt(btMatrix3x3::getIdentity(), startv);
	btTransform endt(btMatrix3x3::getIdentity(), endv);

	if (ray.m_IsRay) {
		btCollisionWorld::ClosestRayResultCallback cb(startv, endv);
		btCollisionWorld::rayTestSingle(startt, endt, object, shape, transform, cb);

		ptr->fraction = cb.m_closestHitFraction;
		ConvertPosToHL(cb.m_hitPointWorld, ptr->endpos);
		ConvertDirectionToHL(cb.m_hitNormalWorld, ptr->plane.normal);
	} else {
	
		ConvertPosToBull(ray.m_Extents, btvec);
		btBoxShape* box = new btBoxShape(btvec);

		btCollisionWorld::ClosestConvexResultCallback cb(startv, endv);
		btCollisionWorld::objectQuerySingle(box, startt, endt, object, shape, transform, cb, 0);

		ptr->fraction = cb.m_closestHitFraction;
		ConvertPosToHL(cb.m_hitPointWorld, ptr->endpos);
		ConvertDirectionToHL(cb.m_hitNormalWorld, ptr->plane.normal);
		delete box;
	}
	delete object;
}

void CPhysicsCollision::TraceCollide(const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	NOT_IMPLEMENTED;
}

bool CPhysicsCollision::IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone) {
	NOT_IMPLEMENTED;
	return false;
}

void CPhysicsCollision::VCollideLoad(vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap) {
	g_ValvePhysicsCollision->VCollideLoad(pOutput, solidCount, pBuffer, size, swap);
	for (int i = 0; i < solidCount; i++) {
		CPhysCollide* ivp = pOutput->solids[i];
		pOutput->solids[i] = (CPhysCollide*)ConvertMeshToBull(ivp);
		g_ValvePhysicsCollision->DestroyCollide(ivp);
	}
}

void CPhysicsCollision::VCollideUnload(vcollide_t *pVCollide) {
	for (int i = 0; i < pVCollide->solidCount; i++) {
		btCollisionShape* shape = (btCollisionShape*)pVCollide->solids[i];
		delete shape;
	}
}

IVPhysicsKeyParser* CPhysicsCollision::VPhysicsKeyParserCreate(const char *pKeyData) {
	return g_ValvePhysicsCollision->VPhysicsKeyParserCreate(pKeyData);
}

void CPhysicsCollision::VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser) {
	g_ValvePhysicsCollision->VPhysicsKeyParserDestroy(pParser);
}

int CPhysicsCollision::CreateDebugMesh(CPhysCollide const *pCollisionModel, Vector **outVerts) {
	NOT_IMPLEMENTED;
	return 0;
}

void CPhysicsCollision::DestroyDebugMesh(int vertCount, Vector *outVerts) {
	NOT_IMPLEMENTED;
}

ICollisionQuery* CPhysicsCollision::CreateQueryModel(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsCollision::DestroyQueryModel(ICollisionQuery *pQuery) {
	NOT_IMPLEMENTED;
}

IPhysicsCollision* CPhysicsCollision::ThreadContextCreate() {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsCollision::ThreadContextDestroy(IPhysicsCollision *pThreadContex) {
	NOT_IMPLEMENTED;
}

CPhysCollide* CPhysicsCollision::CreateVirtualMesh(const virtualmeshparams_t &params) {
	// This doesn't appear to work.
	CPhysCollide* ivp = g_ValvePhysicsCollision->CreateVirtualMesh(params);

	Vector hlvec[3];
	btVector3 btvec[3];
	ICollisionQuery* query = g_ValvePhysicsCollision->CreateQueryModel(ivp);
	int convexcount = query->ConvexCount();
	btCompoundShape* bull = new btCompoundShape;
	for (int convex = 0; convex < convexcount; convex++) {
		btTriangleMesh* btmesh= new btTriangleMesh;
		int triangles = query->TriangleCount(convex);
		for (int i = 0; i < triangles; i++) {
			query->GetTriangleVerts(convex, i, hlvec);
			ConvertPosToBull(hlvec[0], btvec[0]);
			ConvertPosToBull(hlvec[1], btvec[1]);
			ConvertPosToBull(hlvec[2], btvec[2]);
			btmesh->addTriangle(btvec[0], btvec[1], btvec[2], true);
		}
		btBvhTriangleMeshShape* btmeshshape = new btBvhTriangleMeshShape(btmesh, true);
		bull->addChildShape(btTransform::getIdentity(), btmeshshape);
	}

	g_ValvePhysicsCollision->DestroyCollide(ivp);
	return NULL;
}

bool CPhysicsCollision::SupportsVirtualMesh() {
	NOT_IMPLEMENTED;
	return true;
}

bool CPhysicsCollision::GetBBoxCacheSize(int *pCachedSize, int *pCachedCount) {
	NOT_IMPLEMENTED;
	return false;
}

CPolyhedron* CPhysicsCollision::PolyhedronFromConvex(CPhysConvex * const pConvex, bool bUseTempPolyhedron) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsCollision::OutputDebugInfo(const CPhysCollide *pCollide) {
	NOT_IMPLEMENTED;
}

unsigned int CPhysicsCollision::ReadStat(int statID) {
	NOT_IMPLEMENTED;
	return 0;
}

CPhysicsCollision g_PhysicsCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, g_PhysicsCollision);
