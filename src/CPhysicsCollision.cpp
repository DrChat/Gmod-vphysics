#include "StdAfx.h"

#include "CPhysicsCollision.h"
#include "convert.h"
#include "CPhysicsKeyParser.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

//IPhysicsCollision* g_ValvePhysicsCollision = NULL;

CPhysicsCollision::CPhysicsCollision() {

}

CPhysicsCollision::~CPhysicsCollision() {

}

CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	NOT_IMPLEMENTED;
	return NULL;
}

CPhysConvex *CPhysicsCollision::ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance) {
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

CPhysConvex *CPhysicsCollision::BBoxToConvex(const Vector &mins, const Vector &maxs) {
	NOT_IMPLEMENTED;
	return NULL;
}
CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsCollision::ConvexesFromConvexPolygon(const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput) {
	NOT_IMPLEMENTED;
}

CPhysPolysoup *CPhysicsCollision::PolysoupCreate() {
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
	if (!pCollide)
		return collideOrigin;

	return m_traceapi.GetExtent((btCollisionShape *)pCollide, collideOrigin, collideAngles, direction);
}

void CPhysicsCollision::CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles) {
	// Bullet returns very different AABBs than Havok.
	btCollisionShape* shape = (btCollisionShape*)pCollide;

	btVector3 pos, mins, maxs;
	btMatrix3x3 rot;

	ConvertPosToBull(collideOrigin, pos);
	ConvertRotationToBull(collideAngles, rot);
	btTransform transform(rot, pos);

	PhysicsShapeInfo *shapeInfo = (PhysicsShapeInfo*)shape->getUserPointer();
	if (shapeInfo)
		transform *= btTransform(btMatrix3x3::getIdentity(), shapeInfo->massCenter);

	shape->getAabb(transform, mins, maxs);

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

	PhysicsShapeInfo *shapeInfo = (PhysicsShapeInfo*)shape->getUserPointer();
	if (shapeInfo)
		transform *= btTransform(btMatrix3x3::getIdentity(), shapeInfo->massCenter);
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

struct compactsurfaceheader_t {
	int		size;			// Size of the content after this byte
	int		vphysicsID;		// Generally the ASCII for "VPHY" in newer files
	short	version;
	short	modelType;
	int		surfaceSize;
	Vector	dragAxisAreas;
	int		axisMapSize;
};

// old style phy format
struct legacysurfaceheader_t {
	//int		size;
	float	mass_center[3];
	float	rotation_inertia[3];
	float	upper_limit_radius;
	int		max_deviation : 8;
	int		byte_size : 24;
	int		offset_ledgetree_root;
	int		dummy[3]; 			// dummy[2] is "IVPS" or 0
};

// Purpose: Parses a .phy file's collision
// TODO: This function is really expensive! In the future, should we
// generate the data we need and then save it to a mdl.bphy?
void CPhysicsCollision::VCollideLoad(vcollide_t *pOutput, int solidCount, const char *pBuffer, int bufferSize, bool swap) {
	memset(pOutput, 0, sizeof(*pOutput));
	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide *[solidCount];

	int position = 0;
	for (int i = 0; i < solidCount; i++)
	{
		int size = *(int*)(pBuffer + position);

		const char *solid = pBuffer + position;
		pOutput->solids[i] = (CPhysCollide *)solid;
		position += size + 4; // size + sizeof(int)
	}
	pOutput->pKeyValues = (char*)malloc(bufferSize - position);
	memcpy(pOutput->pKeyValues, pBuffer + position, bufferSize - position);

	// Now for the fun part:
	// We have to convert every solid into a bullet solid
	// because havoc solids are saved into a phy file.
	for (int i = 0; i < solidCount; i++)
	{
		const char *solid = (const char*)pOutput->solids[i];

		compactsurfaceheader_t surfaceheader = *(compactsurfaceheader_t *)pOutput->solids[i];
		legacysurfaceheader_t legacyheader = *(legacysurfaceheader_t *)((char *)pOutput->solids[i] + sizeof(compactsurfaceheader_t));

		PhysicsShapeInfo *info = new PhysicsShapeInfo;

		/*
		assert(*(uint32*)solid == 0x59485056); // VPHY
		short version = *(unsigned short*)(solid + 4), type = *(unsigned short*)(solid + 6);
		assert(version == 0x100);
		unsigned long surfacesize = *(uint32*)(solid + 8);
		*/
		
		assert(surfaceheader.vphysicsID == 0x59485056); // == "VPHY"
		assert(surfaceheader.version == 0x100);

		if (surfaceheader.modelType != 0x0)
		{
			pOutput->solids[i] = NULL;
			continue;
		}

		/*
		info->massCenter = btVector3(*(float*)(solid + 28), -*(float*)(solid + 32), -*(float*)(solid + 36));
		assert(*(uint32*)(solid + 72) == 0x53505649); // IVPS
		const char *convexes = solid + 76;
		*/
		
		info->massCenter = btVector3(legacyheader.mass_center[0], -legacyheader.mass_center[1], -legacyheader.mass_center[2]);
		assert(legacyheader.dummy[2] == 0x53505649); // == "IVPS"
		const char *convexes = solid + 80;

		Msg("Mass center: %f %f %f\n", info->massCenter.x(), info->massCenter.y(), info->massCenter.z());
		btCompoundShape *bull = new btCompoundShape();
		bull->setMargin(COLLISION_MARGIN);
		bull->setUserPointer(info);
		int position = 0;
		for (;;) {
			short tricount = *(short*)(convexes + position + 12);
			uint32 vertexoffset = *(uint32*)(convexes + position);
			const char *vertices = convexes + position + vertexoffset;
			assert(convexes + position < vertices);

			//Msg("Convex with %i triangles and %i vertex offset\n", (int)tricount, (int)vertexoffset);

			position += 16;
			btConvexHullShape *mesh = new btConvexHullShape();
			mesh->setMargin(COLLISION_MARGIN);
			for (int j = 0; j < tricount; j++)
			{
				short index1 = *(short*)(convexes + position + 4), index2 = *(short*)(convexes + position + 8), index3 = *(short*)(convexes + position + 12);
				btVector3 vertex1(*(float*)(vertices + index1 * 16), -*(float*)(vertices + index1 * 16 + 4), -*(float*)(vertices + index1 * 16 + 8));
				btVector3 vertex2(*(float*)(vertices + index2 * 16), -*(float*)(vertices + index2 * 16 + 4), -*(float*)(vertices + index2 * 16 + 8));
				btVector3 vertex3(*(float*)(vertices + index3 * 16), -*(float*)(vertices + index3 * 16 + 4), -*(float*)(vertices + index3 * 16 + 8));

				mesh->addPoint(vertex1);
				mesh->addPoint(vertex2);
				mesh->addPoint(vertex3);

				position += 16;
			}

			btShapeHull* hull = new btShapeHull(mesh);
			btScalar margin = mesh->getMargin();
			hull->buildHull(margin); // HOLY FUCK THIS FUNCTION IS EXPENSIVE
			delete mesh;
			mesh = new btConvexHullShape((btScalar*)hull->getVertexPointer(), hull->numVertices());
			mesh->setMargin(COLLISION_MARGIN);

			bull->addChildShape(btTransform(btMatrix3x3::getIdentity(), -info->massCenter), mesh);
			if (convexes + position >= vertices)
				break;
		}
		pOutput->solids[i] = (CPhysCollide*)bull;
	}

	/*g_ValvePhysicsCollision->VCollideLoad(pOutput, solidCount, pBuffer, size, swap);
	for (int i = 0; i < solidCount; i++) {
		CPhysCollide *ivp = pOutput->solids[i];
		if (!ivp) continue;
		pOutput->solids[i] = (CPhysCollide*)ConvertMeshToBull(ivp);
		g_ValvePhysicsCollision->DestroyCollide(ivp);
	}*/
}

void CPhysicsCollision::VCollideUnload(vcollide_t *pVCollide) {
	for (int i = 0; i < pVCollide->solidCount; i++) {
		btCollisionShape* shape = (btCollisionShape*)pVCollide->solids[i];
		delete shape;
	}
	free(pVCollide->pKeyValues);
}

IVPhysicsKeyParser* CPhysicsCollision::VPhysicsKeyParserCreate(const char *pKeyData) {
	return new CPhysicsKeyParser(pKeyData);
}

void CPhysicsCollision::VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser) {
	delete (CPhysicsKeyParser*)pParser;
}

int CPhysicsCollision::CreateDebugMesh(CPhysCollide const *pCollisionModel, Vector **outVerts) {
	int count = 0;
	btCompoundShape *compound = (btCompoundShape*)pCollisionModel;
	for (int i = 0; i < compound->getNumChildShapes(); i++)
	{
		count += ((btConvexHullShape*)compound->getChildShape(i))->getNumVertices();
	}
	*outVerts = new Vector[count];
	int k = 0;
	for (int i = 0; i < compound->getNumChildShapes(); i++)
	{
		btConvexHullShape *hull = (btConvexHullShape*)compound->getChildShape(i);
		for (int j = hull->getNumVertices()-1; j >= 0; j--) // ugh, source wants the vertices in this order or shit begins to draw improperly
		{
			btVector3 pos;
			hull->getVertex(j, pos);
			ConvertPosToHL(pos, (*outVerts)[k++]);
		}
	}
	return count;
}

void CPhysicsCollision::DestroyDebugMesh(int vertCount, Vector *outVerts) {
	delete outVerts;
}

ICollisionQuery* CPhysicsCollision::CreateQueryModel(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED;
	return NULL;
}

void CPhysicsCollision::DestroyQueryModel(ICollisionQuery *pQuery) {
	NOT_IMPLEMENTED;
}

IPhysicsCollision* CPhysicsCollision::ThreadContextCreate() {
	return new CPhysicsCollision;
}

void CPhysicsCollision::ThreadContextDestroy(IPhysicsCollision *pThreadContex) {
	delete (CPhysicsCollision*)pThreadContex;
}

CPhysCollide* CPhysicsCollision::CreateVirtualMesh(const virtualmeshparams_t &params) {
	IVirtualMeshEvent *handler = params.pMeshEventHandler;

	virtualmeshlist_t *pList = new virtualmeshlist_t;
	handler->GetVirtualMesh(params.userData, pList);

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
