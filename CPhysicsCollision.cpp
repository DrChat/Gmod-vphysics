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
	/*pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide*[solidCount];

	int position = 0; 
	for (int i = 0; i < solidCount; i++) {
		uint32 size = *(uint32*)(pBuffer + position);
		pOutput->solids[i] = (CPhysCollide*)malloc(size);
		memcpy(pOutput->solids[i], pBuffer+4, size);
		position += size+4;
	}
	assert(size > position);
	char *keyValues = (char*)malloc(size-position);
	memcpy(keyValues, pBuffer + position, size-position);
	pOutput->pKeyValues = keyValues;

	for (int i = 0; i < solidCount; i++) { // TODO: Stop coding at 3 AM (someone make this cleaner please)
		char *data = (char*)pOutput->solids[i];
		assert(*(uint32*)data == 0x59485056); // VPHY
		short version = *(uint16*)(data+4), type = *(uint16*)(data+6);
		uint32 surfaceSize = *(uint32*)(data+8);
		btVector3 dragAxisAreas(*(float*)(data+12), *(float*)(data+16), *(float*)(data+20));
		int axisMapSize = *(uint32*)(data+24);
		assert(version == 0x100); // TODO: Support other versions of the format? (Portal 2 models had a crash inside VPhysics, they probably use another version. investigate)
		assert(type == 0x0); // TODO: Support other things such as spheres

		Msg("VPHY version %i type %i surface size %i\n", (int)version, (int)type, surfaceSize);

		assert(*(uint32*)(data + 72) == 0x53505649); // IVPS
		char *convexes = data + 76;

		int position = 0, remaining = 0, tris = 0;
		do
		{
			remaining = *(int*)(convexes + position), tris = *(short*)(convexes + position + 12);
			float *vertecies = (float*)(convexes + position + remaining);
			Msg("Found convex with %i tris %i bytes remaining\n", tris, remaining);
			assert(tris * 16 + 16 <= remaining);
			position += 16;
			for (int j = 0; j < tris; j++)
			{
				int index1 = *(short*)(convexes + position + 4), index2 = *(short*)(convexes + position + 8), index3 = *(short*)(convexes + position + 12);
				if (index1 < 256 && index2 < 256 && index3 < 256) // TODO: Figure out why some polys get messed up
				{
					btVector3 v1(vertecies[index1*4], vertecies[index1*4+1], vertecies[index1*4+2]),
						v2(vertecies[index2*4], vertecies[index2*4+1], vertecies[index2*4+2]),
						v3(vertecies[index3*4], vertecies[index3*4+1], vertecies[index3*4+2]);
					Msg("%f %f %f : %f %f %f : %f %f %f\n", v1.x(), -v1.y(), -v1.z(), v2.x(), -v2.y(), -v2.z(), v3.x(), -v3.y(), -v3.z());
				}
				position += 16;
			}
			assert(position <= int(surfaceSize - 48));
		} while (remaining != tris * 16 + 16);

		delete (void*)pOutput->solids[i];
		pOutput->solids[i] = (CPhysCollide*)new btCompoundShape;
	}*/

	g_ValvePhysicsCollision->VCollideLoad(pOutput, solidCount, pBuffer, size, swap);
	for (int i = 0; i < solidCount; i++) {
		CPhysCollide *ivp = pOutput->solids[i];
		pOutput->solids[i] = (CPhysCollide*)ConvertMeshToBull(ivp);
		g_ValvePhysicsCollision->DestroyCollide(ivp);
	}
}

void CPhysicsCollision::VCollideUnload(vcollide_t *pVCollide) {
	for (int i = 0; i < pVCollide->solidCount; i++) {
		btCollisionShape* shape = (btCollisionShape*)pVCollide->solids[i];
		delete shape;
	}
	// FIXME: Uncomment when no longer relying on havok
	//free(pVCollide->pKeyValues);
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
	IVirtualMeshEvent *handler = params.pMeshEventHandler;

	virtualmeshlist_t *pList = new virtualmeshlist_t;
	handler->GetVirtualMesh(params.userData, pList);
	Msg("Virtual mesh: %i vertecies %i triangles\n", pList->vertexCount, pList->triangleCount);

	btTriangleMesh* btmesh= new btTriangleMesh;
	btVector3 btvec[3];
	for (int i = 0; i < pList->triangleCount; i++)
	{
		ConvertPosToBull(pList->pVerts[pList->indices[i*3+0]], btvec[0]);
		ConvertPosToBull(pList->pVerts[pList->indices[i*3+1]], btvec[1]);
		ConvertPosToBull(pList->pVerts[pList->indices[i*3+2]], btvec[2]);
		btmesh->addTriangle(btvec[0], btvec[1], btvec[2], true);
	}
	btBvhTriangleMeshShape* bull = new btBvhTriangleMeshShape(btmesh, true);
	bull->setMargin(COLLISION_MARGIN);
	return (CPhysCollide*)bull;
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
