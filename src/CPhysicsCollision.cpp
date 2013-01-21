#include "StdAfx.h"

#include "CPhysicsCollision.h"
#include "convert.h"
#include "CPhysicsKeyParser.h"

#include "tier0/vprof.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/****************************
* CLASS CCollisionQuery
****************************/

class CCollisionQuery : public ICollisionQuery {
	public:
		CCollisionQuery(btCollisionShape *pShape) {m_pShape = pShape;}
		~CCollisionQuery();

		// number of convex pieces in the whole solid
		int					ConvexCount();
		// triangle count for this convex piece
		int					TriangleCount(int convexIndex);
		// get the stored game data
		uint				GetGameData(int convexIndex);
		// Gets the triangle's verts to an array
		void				GetTriangleVerts(int convexIndex, int triangleIndex, Vector *verts);
		
		// UNDONE: This doesn't work!!!
		void				SetTriangleVerts(int convexIndex, int triangleIndex, const Vector *verts);
		
		// returns the 7-bit material index
		int					GetTriangleMaterialIndex(int convexIndex, int triangleIndex);
		// sets a 7-bit material index for this triangle
		void				SetTriangleMaterialIndex(int convexIndex, int triangleIndex, int index7bits);

	private:
		btCollisionShape *	m_pShape;
};

int CCollisionQuery::ConvexCount() {
	if (m_pShape->isCompound()) {
		btCompoundShape *pShape = (btCompoundShape *)m_pShape;
		return pShape->getNumChildShapes();
	}

	return 0;
}

/****************************
* CLASS CPhysicsCollision
****************************/

// NOTE:
// CPhysCollide is a btCompoundShape
// CPhysConvex is a btConvexHullShape

CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	btConvexHullShape *pConvex = new btConvexHullShape;

	for (int i = 0; i < vertCount; i++) {
		btVector3 vert;
		ConvertPosToBull(*pVerts[i], vert);

		pConvex->addPoint(vert);
	}

	// Optimize the shape.
	btShapeHull *hull = new btShapeHull(pConvex);
	delete pConvex;
	pConvex = new btConvexHullShape((btScalar *)hull->getVertexPointer(), hull->numVertices());
	delete hull;

	return (CPhysConvex *)pConvex;
}

CPhysConvex *CPhysicsCollision::ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance) {
	NOT_IMPLEMENTED
	return NULL;
}

float CPhysicsCollision::ConvexVolume(CPhysConvex *pConvex) {
	NOT_IMPLEMENTED
	return 0;
}

float CPhysicsCollision::ConvexSurfaceArea(CPhysConvex *pConvex) {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsCollision::SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData) {
	NOT_IMPLEMENTED
}

void CPhysicsCollision::ConvexFree(CPhysConvex *pConvex) {
	delete (btConvexHullShape *)pConvex;
}

CPhysConvex *CPhysicsCollision::BBoxToConvex(const Vector &mins, const Vector &maxs) {
	NOT_IMPLEMENTED
	return NULL;
}

CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsCollision::ConvexesFromConvexPolygon(const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput) {
	NOT_IMPLEMENTED
}

CPhysPolysoup *CPhysicsCollision::PolysoupCreate() {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsCollision::PolysoupDestroy(CPhysPolysoup *pSoup) {
	NOT_IMPLEMENTED
}

void CPhysicsCollision::PolysoupAddTriangle(CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits) {
	NOT_IMPLEMENTED
}

CPhysCollide *CPhysicsCollision::ConvertPolysoupToCollide(CPhysPolysoup *pSoup, bool useMOPP) {
	NOT_IMPLEMENTED
	return NULL;
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount) {
	NOT_IMPLEMENTED
	return NULL;
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollideParams(CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsCollision::DestroyCollide(CPhysCollide *pCollide) {
	btCollisionShape *shape = (btCollisionShape *)pCollide;
	delete shape;
}

int CPhysicsCollision::CollideSize(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

int CPhysicsCollision::CollideWrite(char *pDest, CPhysCollide *pCollide, bool bSwap) {
	NOT_IMPLEMENTED
	return 0;
}

CPhysCollide *CPhysicsCollision::UnserializeCollide(char *pBuffer, int size, int index) {
	NOT_IMPLEMENTED
	return NULL;
}

float CPhysicsCollision::CollideVolume(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

float CPhysicsCollision::CollideSurfaceArea(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

// This'll return the farthest possible vector that's still within our collision mesh.
Vector CPhysicsCollision::CollideGetExtent(const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction) {
	if (!pCollide)
		return collideOrigin;

	NOT_IMPLEMENTED
	return collideOrigin;
}

// Called every frame when an object is reported to be asleep in CPhysicsObject::IsAsleep!
void CPhysicsCollision::CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles) {
	if (!pCollide || (!pMins && !pMaxs)) return;

	// Bullet returns very different AABBs than Havok.
	btCollisionShape *shape = (btCollisionShape *)pCollide;

	btVector3 pos, mins, maxs;
	btMatrix3x3 rot;

	ConvertPosToBull(collideOrigin, pos);
	ConvertRotationToBull(collideAngles, rot);
	btTransform transform(rot, pos);

	PhysicsShapeInfo *shapeInfo = (PhysicsShapeInfo *)shape->getUserPointer();
	if (shapeInfo)
		transform *= btTransform(btMatrix3x3::getIdentity(), shapeInfo->massCenter);

	shape->getAabb(transform, mins, maxs);

	if (pMins)
		ConvertPosToHL(mins, *pMins);
	if (pMaxs)
		ConvertPosToHL(maxs, *pMaxs);
}

void CPhysicsCollision::CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter) {
	if (!pCollide || !pOutMassCenter) return;

	btCollisionShape *pShape = (btCollisionShape *)pCollide;
	PhysicsShapeInfo *pInfo = (PhysicsShapeInfo *)pShape->getUserPointer();

	if (pInfo) {
		Vector massCenter;
		ConvertPosToHL(pInfo->massCenter, massCenter);

		*pOutMassCenter = massCenter;
	}
}

void CPhysicsCollision::CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter) {
	if (!pCollide) return;

	btCollisionShape *pShape = (btCollisionShape *)pCollide;
	PhysicsShapeInfo *pInfo = (PhysicsShapeInfo *)pShape->getUserPointer();

	if (pInfo) {
		btVector3 bullMassCenter;
		ConvertPosToBull(massCenter, bullMassCenter);

		pInfo->massCenter = bullMassCenter;
	}
}

Vector CPhysicsCollision::CollideGetOrthographicAreas(const CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return Vector();
}

void CPhysicsCollision::CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas) {
	NOT_IMPLEMENTED
}

int CPhysicsCollision::CollideIndex(const CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

CPhysCollide *CPhysicsCollision::BBoxToCollide(const Vector &mins, const Vector &maxs) {
	if (mins == maxs) return NULL;

	btVector3 btmins, btmaxs;
	ConvertPosToBull(mins, btmins);
	ConvertPosToBull(maxs, btmaxs);
	btVector3 halfsize = (btmaxs - btmins) / 2;

	btBoxShape *box = new btBoxShape(halfsize);
	btCompoundShape *shape = new btCompoundShape;

	btTransform transform(btMatrix3x3::getIdentity(), btmins + halfsize);
	shape->addChildShape(transform, box);

	return (CPhysCollide *)shape;
}

int CPhysicsCollision::GetConvexesUsedInCollideable(const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit) {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsCollision::TraceBox(const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	NOT_IMPLEMENTED
}

void CPhysicsCollision::TraceBox(const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	return TraceBox(ray, MASK_ALL, NULL, pCollide, collideOrigin, collideAngles, ptr);
}

// FIXME: Lack of collision between players and objects might be caused by inaccuracy in this function?
// TODO: Use contentsMask
void CPhysicsCollision::TraceBox(const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	VPROF_BUDGET("CPhysicsCollision::TraceBox", VPROF_BUDGETGROUP_PHYSICS);

	btVector3 btvec;
	btMatrix3x3 btmatrix;
	btCollisionObject *object = new btCollisionObject;
	btCollisionShape *shape = (btCollisionShape *)pCollide;
	object->setCollisionShape(shape);
	ConvertPosToBull(collideOrigin, btvec);
	ConvertRotationToBull(collideAngles, btmatrix);
	btTransform transform(btmatrix, btvec);

	PhysicsShapeInfo *shapeInfo = (PhysicsShapeInfo *)shape->getUserPointer();
	if (shapeInfo)
		transform *= btTransform(btMatrix3x3::getIdentity(), shapeInfo->massCenter);
	object->setWorldTransform(transform);

	btVector3 startv, endv;
	ConvertPosToBull(ray.m_Start, startv);
	ConvertPosToBull(ray.m_Start + ray.m_Delta, endv);
	btTransform startt(btMatrix3x3::getIdentity(), startv);
	btTransform endt(btMatrix3x3::getIdentity(), endv);

	// m_IsRay appears to only be true when the crosshair crosses the AABB of the object.
	// Otherwise, we're doing a box trace.
	// TODO: Box trace doesn't handle trace failures!
	// Does the trace always fail if our object is penetrating the other one?
	if (ray.m_IsRay) {
		btCollisionWorld::ClosestRayResultCallback cb(startv, endv);
		btCollisionWorld::rayTestSingle(startt, endt, object, shape, transform, cb);

		ptr->fraction = cb.m_closestHitFraction;
		ConvertPosToHL(cb.m_hitPointWorld, ptr->endpos);
		ConvertDirectionToHL(cb.m_hitNormalWorld, ptr->plane.normal);
	} else {
		ConvertPosToBull(ray.m_Extents, btvec);
		btBoxShape *box = new btBoxShape(btvec);

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
	btVector3 bullVec;
	btMatrix3x3 bullMatrix;

	// Create the collision object
	btCollisionObject *object = new btCollisionObject;
	btCollisionShape *shape = (btCollisionShape *)pCollide;
	object->setCollisionShape(shape);
	ConvertRotationToBull(collideAngles, bullMatrix);
	ConvertPosToBull(collideOrigin, bullVec);
	btTransform transform(bullMatrix, bullVec);

	PhysicsShapeInfo *shapeInfo = (PhysicsShapeInfo *)shape->getUserPointer();
	if (shapeInfo)
		transform *= btTransform(btMatrix3x3::getIdentity(), shapeInfo->massCenter);
	object->setWorldTransform(transform);

	// Create the sweep collision object
	btCollisionObject *pSweepObject = new btCollisionObject;
	btCollisionShape *pSweepShape = (btCollisionShape *)pSweepCollide;
	pSweepObject->setCollisionShape(pSweepShape);
	ConvertRotationToBull(sweepAngles, bullMatrix);
	btTransform bullSweepTransform(bullMatrix);

	PhysicsShapeInfo *pSweepShapeInfo = (PhysicsShapeInfo *)pSweepShape->getUserPointer();
	if (pSweepShapeInfo)
		bullSweepTransform *= btTransform(btMatrix3x3::getIdentity(), pSweepShapeInfo->massCenter);
	pSweepObject->setWorldTransform(bullSweepTransform);

	btVector3 bullStartVec, bullEndVec;
	ConvertPosToBull(start, bullStartVec);
	ConvertPosToBull(end, bullEndVec);
	btTransform bullStartT(btMatrix3x3::getIdentity(), bullStartVec);
	btTransform bullEndT(btMatrix3x3::getIdentity(), bullEndVec);

	btCollisionWorld::ClosestConvexResultCallback cb(bullStartVec, bullEndVec);

	// Cleanup
	delete object;
	delete pSweepObject;

	NOT_IMPLEMENTED
}

bool CPhysicsCollision::IsBoxIntersectingCone(const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone) {
	NOT_IMPLEMENTED
	return false;
}

// Various structures used in vcollide parsing.
// 28 bytes
struct compactsurfaceheader_t {
	int		vphysicsID;		// Generally the ASCII for "VPHY" in newer files
	short	version;
	short	modelType;
	int		surfaceSize;
	Vector	dragAxisAreas;
	int		axisMapSize;
};

// 16 bytes
// Just like a btCompoundShape.
struct ivpcompactsurface_t {
	float	mass_center[3];
	float	rotation_inertia[3];
	float	upper_limit_radius;
	int		max_deviation : 8;
	int		byte_size : 24;
	int		offset_ledgetree_root;
	int		dummy[3]; 			// dummy[2] is "IVPS" or 0
};

// 16 bytes
// Just like a btConvexShapeHull.
struct ivpcompactledge_t {
	int		c_point_offset; // byte offset from 'this' to (ledge) point array
	union {
		int	ledgetree_node_offset;
		int	client_data;	// if indicates a non terminal ledge
	};
	uint	has_chilren_flag:2;
	int		is_compact_flag:2;  // if false than compact ledge uses points outside this piece of memory
	uint	dummy:4;
	uint	size_div_16:24; 
	short	n_triangles;
	short	for_future_use;
};

// 4 bytes
struct ivpcompactedge_t {
	uint	start_point_index:16;		// point index
	int		opposite_index:15;			// rel to this // maybe extra array, 3 bits more than tri_index/pierce_index
	uint	is_virtual:1;
};

// 16 bytes (4 bytes + 12 bytes edge array)
struct ivpcompacttriangle_t {
	uint				tri_index : 12; // used for upward navigation
	uint				pierce_index : 12;
	uint				material_index : 7;
	uint				is_virtual : 1;	// DrChat; I believe this is just an optimization. Just ignore anything marked is_virtual.
	ivpcompactedge_t	c_three_edges[3];
};

// Purpose: Loads and converts an ivp mesh to a bullet mesh.
void CPhysicsCollision::VCollideLoad(vcollide_t *pOutput, int solidCount, const char *pBuffer, int bufferSize, bool swap) {
	memset(pOutput, 0, sizeof(*pOutput));
	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide *[solidCount];

	int position = 0;
	for (int i = 0; i < solidCount; i++) {
		int size = *(int *)(pBuffer + position);
		position += 4; // Skip the size int.

		pOutput->solids[i] = (CPhysCollide *)(pBuffer + position);
		position += size;
	}
	pOutput->pKeyValues = new char[bufferSize - position];
	memcpy(pOutput->pKeyValues, pBuffer + position, bufferSize - position);

	// swap possibly meaning bitswap?
	DevMsg("VPhysics: VCollideLoad with %d solids, swap is %s\n", solidCount, swap ? "true" : "false");

	// Now for the fun part:
	// We have to convert every solid into a bullet solid
	// because a phy file is made up of ivp solids.
	for (int i = 0; i < solidCount; i++) {
		const char *solid = (const char *)pOutput->solids[i];
		const compactsurfaceheader_t surfaceheader = *(compactsurfaceheader_t *)pOutput->solids[i];
		const ivpcompactsurface_t ivpsurface = *(ivpcompactsurface_t *)((char *)pOutput->solids[i] + sizeof(compactsurfaceheader_t));

		PhysicsShapeInfo *info = new PhysicsShapeInfo;

		// Some checks and warnings (condense these when we want to remove the warnings)
		if (surfaceheader.vphysicsID != MAKEID('V', 'P', 'H', 'Y')) {
			Warning("VPhysics: Could not load a solid! (surfaceheader.vphysicsID != \"VPHY\")\n");
			pOutput->solids[i] = NULL;
			continue;
		}

		if (surfaceheader.version != 0x100) {
			Warning("VPhysics: Could not load a solid! (surfaceheader.version == %X)\n", surfaceheader.version);
			pOutput->solids[i] = NULL;
			continue;
		}

		if (surfaceheader.modelType != 0x0) {
			Warning("VPhysics: Could not load a solid! (surfaceheader.modelType == %X)\n", surfaceheader.modelType);
			pOutput->solids[i] = NULL;
			continue;
		}

		info->massCenter = btVector3(ivpsurface.mass_center[0], -ivpsurface.mass_center[1], -ivpsurface.mass_center[2]);

		// TODO: Support loading of IVP MOPP (is it even used?)
		Assert(ivpsurface.dummy[2] == MAKEID('I', 'V', 'P', 'S'));
		const char *convexes = solid + 76; // Right after ivpsurface

		btCompoundShape *bull = new btCompoundShape();
		bull->setMargin(COLLISION_MARGIN);
		bull->setUserPointer(info);
		int position = 0;

		// Add all of the convex solids to our compound shape.
		while (true) {
			// Structure:
			// ivp_compact_ledge, ivp_compact_triangle[ledge.n_trianges], repeat until vertices
			// Later on are actual edge(vertices) points of each triangle

			// TODO: IVP Solids contain a final ledge which appears to be a convex wrap
			// of any concave models. (Found from phy file props_junk/TrashDumpster02.phy @ 0x000007B0)
			// (Also found from phy file props_c17/FurnitureWashingmachine001a.phy @ 0x000002D0)
			// From my observations of IVP source, they appear to be using qhull internally. They
			// also will run through and mark any points not included in the initial points received
			// as is_virtual(?)

			// TODO: Detailed analysis of dumpster02, as it still appears to have a distorted
			// collision mesh
			const ivpcompactledge_t ivpledge = *(ivpcompactledge_t *)(convexes + position);

			const char *vertices = convexes + position + ivpledge.c_point_offset; // point offset

			// Triangles start after the ivp_compact_ledge
			position += sizeof(ivpcompactledge_t);
			btConvexHullShape *mesh = new btConvexHullShape();
			mesh->setMargin(COLLISION_MARGIN);

			// Add all of the triangles to our mesh.
			for (int j = 0; j < ivpledge.n_triangles; j++) {
				const ivpcompacttriangle_t ivptri = *(ivpcompacttriangle_t *)(convexes + position);
				position += sizeof(ivpcompacttriangle_t);

				if (ivptri.is_virtual)
					continue;

				for (int k = 0; k < 3; k++) {
					short index = ivptri.c_three_edges[k].start_point_index;
					float *verts = (float *)(vertices + index * 16);

					btVector3 vertex(verts[0], -verts[1], -verts[2]);
					mesh->addPoint(vertex);
				}
			}

			// Not enough points to make a triangle.
			if (mesh->getNumPoints() >= 3) {
				bull->addChildShape(btTransform(btMatrix3x3::getIdentity(), -info->massCenter), mesh); // Move by opposite of center of mass since bullet takes our origin as the center of mass.
			} else {
				delete mesh;
			}

			if (convexes + position >= vertices)
				break;
		}

		pOutput->solids[i] = (CPhysCollide *)bull;
	}
}

void CPhysicsCollision::VCollideUnload(vcollide_t *pVCollide) {
	for (int i = 0; i < pVCollide->solidCount; i++) {
		btCollisionShape *pShape = (btCollisionShape *)pVCollide->solids[i];

		// Compound shape? Delete all of it's children.
		if (pShape->isCompound()) {
			btCompoundShape *pCompound = (btCompoundShape *)pShape;
			int numChildShapes = pCompound->getNumChildShapes();
			for (int j = 0; j < numChildShapes; j++) {
				delete (btCollisionShape *)pCompound->getChildShape(j);
			}

			// Also delete our PhysicsShapeInfo thing.
			delete (PhysicsShapeInfo *)pCompound->getUserPointer();
		}

		delete pShape;
		pVCollide->solids[i] = NULL;
	}

	delete pVCollide->pKeyValues;
}

IVPhysicsKeyParser *CPhysicsCollision::VPhysicsKeyParserCreate(const char *pKeyData) {
	return new CPhysicsKeyParser(pKeyData);
}

void CPhysicsCollision::VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser) {
	delete (CPhysicsKeyParser *)pParser;
}

int CPhysicsCollision::CreateDebugMesh(CPhysCollide const *pCollisionModel, Vector **outVerts) {
	int count = 0;
	btCompoundShape *compound = (btCompoundShape *)pCollisionModel;
	for (int i = 0; i < compound->getNumChildShapes(); i++)
		count += ((btConvexHullShape *)compound->getChildShape(i))->getNumVertices();
	
	*outVerts = new Vector[count];
	int k = 0;

	// BUG: This doesn't work on concave meshes! (Lots of lines connecting far apart vertices)
	for (int i = 0; i < compound->getNumChildShapes(); i++) {
		btConvexHullShape *hull = (btConvexHullShape *)compound->getChildShape(i);
		for (int j = hull->getNumVertices()-1; j >= 0; j--) { // ugh, source wants the vertices in this order or shit begins to draw improperly
			btVector3 pos;
			hull->getVertex(j, pos);
			ConvertPosToHL(pos, (*outVerts)[k++]);
		}
	}
	return count;
}

void CPhysicsCollision::DestroyDebugMesh(int vertCount, Vector *outVerts) {
	delete [] outVerts;
}

ICollisionQuery *CPhysicsCollision::CreateQueryModel(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return NULL;
	//return new CCollisionQuery();
}

void CPhysicsCollision::DestroyQueryModel(ICollisionQuery *pQuery) {
	NOT_IMPLEMENTED
	// delete (CCollisionQuery *)pQuery;
}

IPhysicsCollision *CPhysicsCollision::ThreadContextCreate() {
	return new CPhysicsCollision;
}

void CPhysicsCollision::ThreadContextDestroy(IPhysicsCollision *pThreadContex) {
	delete (CPhysicsCollision *)pThreadContex;
}

CPhysCollide *CPhysicsCollision::CreateVirtualMesh(const virtualmeshparams_t &params) {
	IVirtualMeshEvent *handler = params.pMeshEventHandler;

	virtualmeshlist_t *pList = new virtualmeshlist_t;
	handler->GetVirtualMesh(params.userData, pList);

	btTriangleMesh *btmesh = new btTriangleMesh;
	btVector3 btvec[3];
	for (int i = 0; i < pList->triangleCount; i++) {
		ConvertPosToBull(pList->pVerts[pList->indices[i*3+0]], btvec[0]);
		ConvertPosToBull(pList->pVerts[pList->indices[i*3+1]], btvec[1]);
		ConvertPosToBull(pList->pVerts[pList->indices[i*3+2]], btvec[2]);
		btmesh->addTriangle(btvec[0], btvec[1], btvec[2], true);
	}

	delete pList;

	btBvhTriangleMeshShape *bull = new btBvhTriangleMeshShape(btmesh, true);
	bull->setMargin(COLLISION_MARGIN);
	return (CPhysCollide *)bull;
}

// Function only called once in server.dll - seems pretty pointless for valve to make this function.
bool CPhysicsCollision::SupportsVirtualMesh() {
	return true;
}

bool CPhysicsCollision::GetBBoxCacheSize(int *pCachedSize, int *pCachedCount) {
	NOT_IMPLEMENTED
	return false;
}

CPolyhedron *CPhysicsCollision::PolyhedronFromConvex(CPhysConvex *const pConvex, bool bUseTempPolyhedron) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsCollision::OutputDebugInfo(const CPhysCollide *pCollide) {
	Msg("Congratulations! You have found the output of CPhysicsCollision::OutputDebugInfo!\nInform the developers of the command you used to generate this message!\n");
	NOT_IMPLEMENTED
}

unsigned int CPhysicsCollision::ReadStat(int statID) {
	NOT_IMPLEMENTED
	return 0;
}

CPhysicsCollision g_PhysicsCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, g_PhysicsCollision);
