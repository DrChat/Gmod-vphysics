#include "StdAfx.h"

#include <vphysics/virtualmesh.h>
#include <cmodel.h>

#include "Physics_Collision.h"
#include "convert.h"
#include "Physics_KeyParser.h"
#include "phydata.h"

#include "tier0/vprof.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

#define COLLISION_MARGIN 0.004 // 4 mm

// Use btConvexTriangleMeshShape instead of btConvexHullShape?
//#define USE_CONVEX_TRIANGLES

// lol hack
extern IVPhysicsDebugOverlay *g_pDebugOverlay;

/****************************
* CLASS CCollisionQuery
****************************/

// FIXME: We don't use triangles to represent shapes internally!
// Low priority, not even used ingame
class CCollisionQuery : public ICollisionQuery {
	public:
		CCollisionQuery(btCollisionShape *pShape) {m_pShape = pShape;}

		// number of convex pieces in the whole solid
		int					ConvexCount();
		// triangle count for this convex piece
		int					TriangleCount(int convexIndex);
		// get the stored game data
		uint				GetGameData(int convexIndex);
		// Gets the triangle's verts to an array
		void				GetTriangleVerts(int convexIndex, int triangleIndex, Vector *verts);
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

int CCollisionQuery::TriangleCount(int convexIndex) {
	NOT_IMPLEMENTED
	return 0;
}

unsigned int CCollisionQuery::GetGameData(int convexIndex) {
	NOT_IMPLEMENTED
	return 0;
}

void CCollisionQuery::GetTriangleVerts(int convexIndex, int triangleIndex, Vector *verts) {
	NOT_IMPLEMENTED
}

void CCollisionQuery::SetTriangleVerts(int convexIndex, int triangleIndex, const Vector *verts) {
	NOT_IMPLEMENTED
}

int CCollisionQuery::GetTriangleMaterialIndex(int convexIndex, int triangleIndex) {
	NOT_IMPLEMENTED
	return 0;
}

void CCollisionQuery::SetTriangleMaterialIndex(int convexIndex, int triangleIndex, int index7bits) {
	NOT_IMPLEMENTED
}

/****************************
* CLASS CPhysicsCollision
****************************/

// NOTE:
// CPhysCollide is usually a btCompoundShape
// CPhysConvex is usually a btConvexHullShape

#define VPHYSICS_ID					MAKEID('V', 'P', 'H', 'Y')
#define IVP_COMPACT_SURFACE_ID		MAKEID('I', 'V', 'P', 'S')
#define IVP_COMPACT_MOPP_ID			MAKEID('M', 'O', 'P', 'P')

CPhysicsCollision::CPhysicsCollision() {
	// Default to old behavior
	EnableBBoxCache(true);
}

CPhysicsCollision::~CPhysicsCollision() {
	ClearBBoxCache();
}

CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	if (!pVerts) return NULL;

	btConvexHullShape *pConvex = new btConvexHullShape;

	for (int i = 0; i < vertCount; i++) {
		btVector3 vert;
		ConvertPosToBull(*pVerts[i], vert);

		pConvex->addPoint(vert);
	}

	return (CPhysConvex *)pConvex;
}

CPhysConvex *CPhysicsCollision::ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance) {
	NOT_IMPLEMENTED
	return NULL;
}

float CPhysicsCollision::ConvexVolume(CPhysConvex *pConvex) {
	if (!pConvex) return 0;

	NOT_IMPLEMENTED
	return 0;
}

float CPhysicsCollision::ConvexSurfaceArea(CPhysConvex *pConvex) {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsCollision::ConvexFree(CPhysConvex *pConvex) {
	delete (btConvexShape *)pConvex;
}

void CPhysicsCollision::SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData) {
	NOT_IMPLEMENTED
}

CPolyhedron *CPhysicsCollision::PolyhedronFromConvex(CPhysConvex *const pConvex, bool bUseTempPolyhedron) {
	NOT_IMPLEMENTED
	return NULL;
}

CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsCollision::ConvexesFromConvexPolygon(const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **ppOutput) {
	NOT_IMPLEMENTED
}

// TODO: Support this, gmod lua Entity:PhysicsInitMultiConvex uses this!
// IVP internally used QHull to generate the convexes.
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

CPhysCollide *CPhysicsCollision::ConvertConvexToCollide(CPhysConvex **ppConvex, int convexCount) {
	if (convexCount == 0) return NULL;

	btCompoundShape *pCompound = new btCompoundShape;
	for (int i = 0; i < convexCount; i++) {
		btCollisionShape *pShape = (btCollisionShape *)ppConvex[i];
		pCompound->addChildShape(btTransform::getIdentity(), pShape);
	}

	return (CPhysCollide *)pCompound;
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollideParams(CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams) {
	NOT_IMPLEMENTED
	return ConvertConvexToCollide(pConvex, convexCount);
}

void CPhysicsCollision::AddConvexToCollide(CPhysCollide *pCollide, const CPhysConvex *pConvex, const matrix3x4_t *xform) {
	if (!pCollide || !pConvex) return;

	if (((btCollisionShape *)pCollide)->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pCollide;
		btCollisionShape *pShape = (btCollisionShape *)pConvex;

		btTransform trans = btTransform::getIdentity();
		if (xform) {
			ConvertMatrixToBull(*xform, trans);
		}

		pCompound->addChildShape(trans, pShape);
	}
}

void CPhysicsCollision::RemoveConvexFromCollide(CPhysCollide *pCollide, const CPhysConvex *pConvex) {
	if (!pCollide || !pConvex) return;

	if (((btCollisionShape *)pCollide)->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pCollide;
		btCollisionShape *pShape = (btCollisionShape *)pConvex;

		pCompound->removeChildShape(pShape);
	}
}

// Purpose: Recursive function that will go through all compounds and delete their children
void DestroyCompoundShape(btCompoundShape *pCompound) {
	if (!pCompound) return;
	int numChildShapes = pCompound->getNumChildShapes();

	// We're looping in reverse because we're removing objects from the compound shape.
	for (int i = numChildShapes - 1; i >= 0; i--) {
		btCollisionShape *pShape = pCompound->getChildShape(i);

		pCompound->removeChildShapeByIndex(i);
		g_PhysicsCollision.DestroyCollide((CPhysCollide *)pShape);
	}

	delete (PhysicsShapeInfo *)pCompound->getUserPointer();
	delete pCompound;
}

void CPhysicsCollision::DestroyCollide(CPhysCollide *pCollide) {
	if (!pCollide || IsCachedBBox(pCollide)) return;

	btCollisionShape *pShape = (btCollisionShape *)pCollide;

	// Compound shape? Delete all of its children.
	if (pShape->isCompound()) {
		DestroyCompoundShape((btCompoundShape *)pShape);
	} else if (pShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
		// Delete the striding mesh interface (which we allocate)
		btStridingMeshInterface *pMesh = ((btTriangleMeshShape *)pShape)->getMeshInterface();
		delete pMesh;

		delete pShape;
	} else if (pShape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
		btStridingMeshInterface *pMesh = ((btConvexTriangleMeshShape *)pShape)->getMeshInterface();
		delete pMesh;

		delete pShape;
	} else {
		delete pShape;
	}
}

int CPhysicsCollision::CollideSize(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

// TODO: Should we write a binary-compatible version with IVP or use our own format?
int CPhysicsCollision::CollideWrite(char *pDest, CPhysCollide *pCollide, bool swap) {
	NOT_IMPLEMENTED
	return 0;
}

CPhysCollide *CPhysicsCollision::UnserializeCollide(char *pBuffer, int size, int index) {
	NOT_IMPLEMENTED
	return NULL;
}

// TODO: Use qhull
float CPhysicsCollision::CollideVolume(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

// TODO: Use qhull
float CPhysicsCollision::CollideSurfaceArea(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

// This'll return the farthest possible vector that's still within our collision mesh.
// direction argument is a normalized Vector, literally a direction. (to/from the center?)
// TODO: Methods on doing this?
// Does bullet provide any APIs for doing this?
// Should we do a raytrace that starts outside of our mesh and goes inwards?
// From what I gather, valve does this by going through the vertices and returning the one with the nearest dot product to the direction.
Vector CPhysicsCollision::CollideGetExtent(const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction) {
	VPROF_BUDGET("CPhysicsCollision::CollideGetExtent", VPROF_BUDGETGROUP_PHYSICS);
	if (!pCollide) return collideOrigin;

	btCollisionShape *pShape = (btCollisionShape *)pCollide;
	btCollisionObject *pObject = new btCollisionObject;
	pObject->setCollisionShape(pShape);

	// Setup our position
	btVector3 bullPos;
	btMatrix3x3 bullAng;
	ConvertPosToBull(collideOrigin, bullPos);
	ConvertRotationToBull(collideAngles, bullAng);
	btTransform trans(bullAng, bullPos);

	// Compensate for mass offset.
	PhysicsShapeInfo *pInfo = (PhysicsShapeInfo *)pShape->getUserPointer();
	if (pInfo)
		trans *= btTransform(btMatrix3x3::getIdentity(), pInfo->massCenter);

	pObject->setWorldTransform(trans);

	// Cleanup
	delete pObject;

	NOT_IMPLEMENTED
	return collideOrigin;
}

void CPhysicsCollision::CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles) {
	VPROF_BUDGET("CPhysicsCollision::CollideGetAABB", VPROF_BUDGETGROUP_PHYSICS);
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
		transform.setOrigin(transform.getOrigin() + shapeInfo->massCenter);

	shape->getAabb(transform, mins, maxs);

	Vector tMins, tMaxs;
	ConvertAABBToHL(mins, maxs, tMins, tMaxs);

	if (pMins)
		*pMins = tMins;

	if (pMaxs)
		*pMaxs = tMaxs;
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

		// Since mass centers are kind of a hack in our implementation, take care of updating the compound shape's children.
		// FIXME: May cause some issues with rigid bodies "moving" around, or something.
		btVector3 offset = bullMassCenter - pInfo->massCenter;
		if (pShape->isCompound()) {
			btCompoundShape *pCompound = (btCompoundShape *)pShape;
			for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
				btTransform childTrans = pCompound->getChildTransform(i);
				childTrans.setOrigin(childTrans.getOrigin() + offset);
				pCompound->updateChildTransform(i, childTrans);
			}
		}

		pInfo->massCenter = bullMassCenter;
	}
}

Vector CPhysicsCollision::CollideGetOrthographicAreas(const CPhysCollide *pCollide) {
	// What is this?
	NOT_IMPLEMENTED
	return Vector(1, 1, 1); // Documentation says we will return 1,1,1 if ortho areas undefined
}

void CPhysicsCollision::CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas) {
	NOT_IMPLEMENTED
}

void CPhysicsCollision::CollideSetScale(CPhysCollide *pCollide, const Vector &scale) {
	if (!pCollide) return;

	if (((btCollisionShape *)pCollide)->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pCollide;

		btVector3 bullScale;
		ConvertDirectionToBull(scale, bullScale);

		pCompound->setLocalScaling(bullScale);
	}
}

void CPhysicsCollision::CollideGetScale(const CPhysCollide *pCollide, Vector &out) {
	if (!pCollide) return;

	if (((btCollisionShape *)pCollide)->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pCollide;

		btVector3 scale = pCompound->getLocalScaling();
		ConvertDirectionToHL(scale, out);
	}
}

int CPhysicsCollision::CollideIndex(const CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

int CPhysicsCollision::GetConvexesUsedInCollideable(const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit) {
	const btCollisionShape *pShape = (btCollisionShape *)pCollideable;
	if (!pShape->isCompound()) return 0;

	const btCompoundShape *pCompound = (btCompoundShape *)pShape;
	int numSolids = pCompound->getNumChildShapes();
	for (int i = 0; i < numSolids && i < iOutputArrayLimit; i++) {
		const btCollisionShape *pConvex = pCompound->getChildShape(i);
		pOutputArray[i] = (CPhysConvex *)pConvex;
	}

	return numSolids > iOutputArrayLimit ? iOutputArrayLimit : numSolids;
}

CPhysCollide *CPhysicsCollision::GetCachedBBox(const Vector &mins, const Vector &maxs) {
	for (int i = 0; i < m_bboxCache.Count(); i++) {
		bboxcache_t &cache = m_bboxCache[i];

		if (cache.mins == mins && cache.maxs == maxs)
			return cache.pCollide;
	}

	return NULL;
}

void CPhysicsCollision::AddCachedBBox(CPhysCollide *pModel, const Vector &mins, const Vector &maxs) {
	int idx = m_bboxCache.AddToTail();
	bboxcache_t &cache = m_bboxCache[idx];
	cache.pCollide = pModel;
	cache.mins = mins;
	cache.maxs = maxs;
}

bool CPhysicsCollision::IsCachedBBox(CPhysCollide *pModel) {
	for (int i = 0; i < m_bboxCache.Count(); i++) {
		bboxcache_t &cache = m_bboxCache[i];

		if (cache.pCollide == pModel)
			return true;
	}

	return false;
}

void CPhysicsCollision::ClearBBoxCache() {
	for (int i = m_bboxCache.Count() - 1; i >= 0; i--) {
		bboxcache_t &cache = m_bboxCache[i];

		// Remove the cache first so DestroyCollide doesn't stop.
		CPhysCollide *pCollide = cache.pCollide;
		m_bboxCache.Remove(i);

		DestroyCollide(pCollide);
	}
}

bool CPhysicsCollision::GetBBoxCacheSize(int *pCachedSize, int *pCachedCount) {
	// FIXME: Probably not correct. (returns the same number for both outputs)
	if (pCachedSize)
		*pCachedSize = m_bboxCache.Size();

	if (pCachedCount)
		*pCachedCount = m_bboxCache.Count();

	// What do we return?
	return false;
}

void CPhysicsCollision::EnableBBoxCache(bool enable) {
	m_enableBBoxCache = enable;
}

bool CPhysicsCollision::IsBBoxCacheEnabled() {
	return m_enableBBoxCache;
}

CPhysConvex *CPhysicsCollision::BBoxToConvex(const Vector &mins, const Vector &maxs) {
	if (mins == maxs) return NULL;

	btVector3 btmins, btmaxs;
	ConvertAABBToBull(mins, maxs, btmins, btmaxs);
	btVector3 halfExtents = (btmaxs - btmins) / 2;

	btBoxShape *box = new btBoxShape(halfExtents);

	return (CPhysConvex *)box;
}

CPhysCollide *CPhysicsCollision::BBoxToCollide(const Vector &mins, const Vector &maxs) {
	// consult with the bbox cache first (this is old vphysics behavior)
	if (m_enableBBoxCache) {
		CPhysCollide *pCached = GetCachedBBox(mins, maxs);
		if (pCached)
			return pCached;
	}

	CPhysConvex *pConvex = BBoxToConvex(mins, maxs);
	if (!pConvex) return NULL;

	btCompoundShape *pCompound = new btCompoundShape;
	pCompound->setMargin(COLLISION_MARGIN);

	btVector3 btmins, btmaxs;
	ConvertAABBToBull(mins, maxs, btmins, btmaxs);

	btVector3 halfExtents = (btmaxs - btmins) / 2;
	pCompound->addChildShape(btTransform(btMatrix3x3::getIdentity(), btmins + halfExtents), (btCollisionShape *)pConvex);

	if (m_enableBBoxCache)
		AddCachedBBox((CPhysCollide *)pCompound, mins, maxs);

	return (CPhysCollide *)pCompound;
}

CPhysConvex *CPhysicsCollision::CylinderToConvex(const Vector &mins, const Vector &maxs) {
	if (mins == maxs) return NULL;

	btVector3 btmins, btmaxs;
	ConvertAABBToBull(mins, maxs, btmins, btmaxs);
	btVector3 halfSize = (btmaxs - btmins) / 2;

	btCylinderShape *pShape = new btCylinderShape(halfSize);

	return (CPhysConvex *)pShape;
}

CPhysConvex *CPhysicsCollision::ConeToConvex(const float radius, const float height) {
	btConeShape *pShape = new btConeShape(ConvertDistanceToBull(radius), ConvertDistanceToBull(height));
	return (CPhysConvex *)pShape;
}

CPhysConvex *CPhysicsCollision::SphereToConvex(const float radius) {
	if (radius <= 0) return NULL;

	btSphereShape *pShape = new btSphereShape(ConvertDistanceToBull(radius));
	return (CPhysConvex *)pShape;
}

void CPhysicsCollision::TraceBox(const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	Ray_t ray;
	ray.Init(start, end, mins, maxs);
	return TraceBox(ray, pCollide, collideOrigin, collideAngles, ptr);
}

void CPhysicsCollision::TraceBox(const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	return TraceBox(ray, MASK_ALL, NULL, pCollide, collideOrigin, collideAngles, ptr);
}

static ConVar vphysics_visualizetraces("vphysics_visualizetraces", "0", FCVAR_CHEAT, "Visualize physics traces");

// TODO: Use contentsMask
void CPhysicsCollision::TraceBox(const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	VPROF_BUDGET("CPhysicsCollision::TraceBox", VPROF_BUDGETGROUP_PHYSICS);

	// 2 Variables used mainly for converting units.
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

	// Add start offset to the returned trace only.
	ptr->startpos = ray.m_Start + ray.m_StartOffset;

	btTransform startt(btMatrix3x3::getIdentity(), startv);
	btTransform endt(btMatrix3x3::getIdentity(), endv);

	// Single line trace must be supported in TraceBox? Yep, you betcha.
	// FIXME: We can't use frac == 0 to determine if the trace was started in a solid! Need to detect this separately.
	if (ray.m_IsRay) {
		if (ray.m_Delta.Length() != 0) {
			btCollisionWorld::ClosestRayResultCallback cb(startv, endv);
			btCollisionWorld::rayTestSingle(startt, endt, object, shape, transform, cb);

			ptr->fraction = cb.m_closestHitFraction;

			// Data is uninitialized if frac is 1
			if (cb.m_closestHitFraction < 1.0) {
				if (cb.m_closestHitFraction == 0.f) {
					ptr->startsolid = true;
					ptr->allsolid = true;
					ptr->fraction = 0;

					ptr->endpos = ptr->startpos;
				} else {
					ConvertDirectionToHL(cb.m_hitNormalWorld, ptr->plane.normal);
					ConvertPosToHL(cb.m_hitPointWorld, ptr->endpos);
				}
			} else {
				ptr->endpos = ptr->startpos + ray.m_Delta;
			}

			if (vphysics_visualizetraces.GetBool() && g_pDebugOverlay) {
				g_pDebugOverlay->AddLineOverlay(ptr->startpos, ptr->endpos, 0, 0, 255, false, 0.f);
			}
		} else {
			// For whatever reason...
			ptr->startsolid = true;
			ptr->allsolid = true;
			ptr->fraction = 0;

			ptr->endpos = ptr->startpos;
		}
	} else if (ray.m_IsSwept) {
		// Box trace!
		if (vphysics_visualizetraces.GetBool() && g_pDebugOverlay) {
			// Trace start box
			g_pDebugOverlay->AddBoxOverlay(ray.m_Start, -ray.m_Extents, ray.m_Extents, QAngle(0, 0, 0), 255, 0, 0, 50, 0.0f);

			// End trace box
			g_pDebugOverlay->AddBoxOverlay(ray.m_Start + ray.m_Delta, -ray.m_Extents, ray.m_Extents, QAngle(0, 0, 0), 0, 0, 255, 50, 0.0f);
		}

		if (ray.m_Delta.Length() != 0) {
			// extents are half extents, compatible with bullet.
			ConvertPosToBull(ray.m_Extents, btvec);
			btBoxShape *box = new btBoxShape(btvec.absolute());

			btCollisionWorld::ClosestConvexResultCallback cb(startv, endv);
			btCollisionWorld::objectQuerySingle(box, startt, endt, object, shape, transform, cb, 0.001f);

			ptr->fraction = cb.m_closestHitFraction;
			ptr->endpos = ptr->startpos + (ray.m_Delta * ptr->fraction);

			// Data is uninitialized if frac is 1
			if (cb.m_closestHitFraction < 1.0) {
				// BUG: We need to have a fraction atleast a bit bigger than 0 if not started inside object, or the game will always
				// think the player is stuck if walking next to or on top of a physics ent
				// May have to refactor bullet code to have a cb.m_startInSolid flag?
				if (cb.m_closestHitFraction == 0.f) {
					ptr->startsolid = true;
					ptr->allsolid = true;
					ptr->fraction = 0;
				} else {
					ConvertDirectionToHL(cb.m_hitNormalWorld, ptr->plane.normal);
				}

				if (vphysics_visualizetraces.GetBool() && g_pDebugOverlay) {
					if (!ptr->allsolid) {
						Vector lineEnd = ptr->endpos + ptr->plane.normal * 32;
						g_pDebugOverlay->AddLineOverlay(ptr->endpos, lineEnd, 0, 255, 0, false, 0.0f);
					}

					if (ptr->startsolid) {
						g_pDebugOverlay->AddTextOverlay(ptr->endpos, 0, 0.f, "Trace started in solid!");
					}
				}
			}

			delete box;
		} else {
			// For whatever reason...
			ptr->startsolid = true;
			ptr->allsolid = true;
			ptr->fraction = 0;

			ptr->endpos = ptr->startpos;
		}
	}

	delete object;
}

void CPhysicsCollision::TraceCollide(const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace) {
	/*
	btVector3 bullVec;
	btMatrix3x3 bullMatrix;

	// Create the collision object (object to be traced against)
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

	btVector3 bullStartVec, bullEndVec;
	ConvertPosToBull(start, bullStartVec);
	ConvertPosToBull(end, bullEndVec);
	btTransform bullStartT(btMatrix3x3::getIdentity(), bullStartVec);
	btTransform bullEndT(btMatrix3x3::getIdentity(), bullEndVec);

	btCollisionShape *pSweepShape = (btCollisionShape *)pSweepCollide;
	if (pSweepShape->isCompound()) {

	}

	pTrace->fraction = cb.m_closestHitFraction;
	if (cb.m_closestHitFraction < 1.f) {
		ConvertPosToHL(cb.m_hitPointWorld, pTrace->endpos);
		ConvertDirectionToHL(cb.m_hitNormalWorld, pTrace->plane.normal);
	}

	// Cleanup
	delete object;
	*/

	NOT_IMPLEMENTED
}

bool CPhysicsCollision::IsBoxIntersectingCone(const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &truncatedCone) {
	btVector3 boxMins, boxMaxs;
	ConvertAABBToBull(boxAbsMins, boxAbsMaxs, boxMins, boxMaxs);
	btVector3 boxHalfExtents = (boxMaxs - boxMins) / 2;

	btBoxShape *box = new btBoxShape(boxHalfExtents);
	btTransform boxTrans = btTransform::getIdentity();
	boxTrans.setOrigin(boxMins + boxHalfExtents);
	
	// cone
	btScalar coneHeight = ConvertDistanceToBull(truncatedCone.h);
	btScalar coneRadius = btTan(DEG2RAD(truncatedCone.theta)) * coneHeight; // FIXME: Does the theta correspond to the radius or diameter of the bottom?

	btConeShape *cone = new btConeShape(coneRadius, coneHeight);

	// TODO: Find a static function to do a contact pair test

	// Cleanup
	delete box;
	delete cone;

	NOT_IMPLEMENTED
	return false;
}

// Purpose: Recursive function that goes through the entire ledge tree and adds ledges
static void GetAllLedges(const ivpcompactledgenode_t *node, CUtlVector<const ivpcompactledge_t *> *vecOut) {
	if (!node || !vecOut) return;

	if (node->IsTerminal()) {
		vecOut->AddToTail(node->GetCompactLedge());
	} else {
		const ivpcompactledgenode_t *rs = node->GetRightSon();
		const ivpcompactledgenode_t *ls = node->GetLeftSon();
		GetAllLedges(rs, vecOut);
		GetAllLedges(ls, vecOut);
	}
}

static btCollisionShape *LoadMOPP(CPhysCollide *pSolid, bool swap) {
	// Parse MOPP surface header
	const ivpcompactmopp_t *ivpmopp = (ivpcompactmopp_t *)((char *)pSolid + sizeof(compactsurfaceheader_t));

	// valve vphysics casts pSolid to a ivpcompactsurface_t and then compares dummy[2] to this id
	if (ivpmopp->dummy != IVP_COMPACT_MOPP_ID) {
		return NULL;
	}

	// FIXME: Our mopp header struct is incorrect

	//PhysicsShapeInfo *pInfo = new PhysicsShapeInfo;
	//ConvertIVPPosToBull(ivpmopp->mass_center, pInfo->massCenter);

	//btCompoundShape *pCompound = new btCompoundShape;
	//pCompound->setMargin(COLLISION_MARGIN);
	//pCompound->setUserPointer(pInfo);

	// Add up all of the ledges
	CUtlVector<const ivpcompactledge_t *> ledges;
	//GetAllLedges((const ivpcompactledgenode_t *)((char *)ivpmopp + ivpmopp->offset_ledgetree_root), &ledges);

	return NULL;
}

static btCollisionShape *LoadIVPS(CPhysCollide *pSolid, bool swap) {
	// Parse IVP Surface header (which is right after the compact surface header)
	const ivpcompactsurface_t *ivpsurface = (ivpcompactsurface_t *)((char *)pSolid + sizeof(compactsurfaceheader_t));

	if (ivpsurface->dummy[2] != IVP_COMPACT_SURFACE_ID) {
		return NULL;
	}

	// Store info about the mass center for later use.
	PhysicsShapeInfo *info = new PhysicsShapeInfo;
	ConvertIVPPosToBull(ivpsurface->mass_center, info->massCenter);

	btCompoundShape *pCompound = new btCompoundShape;
	pCompound->setMargin(COLLISION_MARGIN);
	pCompound->setUserPointer(info);

	// Add all of the ledges up
	CUtlVector<const ivpcompactledge_t *> ledges;
	GetAllLedges((const ivpcompactledgenode_t *)((char *)ivpsurface + ivpsurface->offset_ledgetree_root), &ledges);

	for (int j = 0; j < ledges.Count(); j++) {
		const ivpcompactledge_t *ledge = ledges[j];

		// Large array of all the vertices
		const char *vertices = (const char *)ledge + ledge->c_point_offset;

		if (ledge->n_triangles > 0) {
#ifdef USE_CONVEX_TRIANGLES
			btTriangleMesh *pMesh = new btTriangleMesh;

			const ivpcompacttriangle_t *tris = (ivpcompacttriangle_t *)ledge + 1;
			for (int k = 0; k < ledge->n_triangles; k++) {
				Assert(k == tris[k].tri_index);

				btVector3 verts[3];

				for (int l = 0; l < 3; l++) {
					short idx = tris[k].c_three_edges[l].start_point_index;
					float *ivpvert = (float *)(vertices + idx * 16);

					ConvertIVPPosToBull(ivpvert, verts[l]);
				}

				pMesh->addTriangle(verts[0], verts[1], verts[2], true);
			}

			btConvexTriangleMeshShape *pShape = new btConvexTriangleMeshShape(pMesh);

			btTransform trans(btMatrix3x3::getIdentity(), -info->massCenter);
			pCompound->addChildShape(trans, pShape);
#else
			btConvexHullShape *pConvex = new btConvexHullShape;
			pConvex->setMargin(COLLISION_MARGIN);

			const ivpcompacttriangle_t *tris = (ivpcompacttriangle_t *)(ledge + 1);

			// This code will find all unique indexes and add them to an array. This avoids
			// adding duplicate points to the convex hull shape (triangle edges can share a vertex)
			// If you find a better way you can replace this!
			CUtlVector<short> indexes;

			for (int k = 0; k < ledge->n_triangles; k++) {
				Assert((uint)k == tris[k].tri_index);

				for (int l = 0; l < 3; l++) {
					short index = tris[k].c_three_edges[l].start_point_index;

					bool shouldAdd = true;
					for (int m = 0; m < indexes.Count(); m++) {
						if (indexes[m] == index) {
							shouldAdd = false;
							break;
						}
					}

					if (shouldAdd) {
						indexes.AddToTail(index);
					}
				}
			}

			for (int k = 0; k < indexes.Count(); k++) {
				short index = indexes[k];

				float *ivpvert = (float *)(vertices + index * 16); // 16 is sizeof(ivp aligned vector)

				btVector3 vertex;
				ConvertIVPPosToBull(ivpvert, vertex);
				pConvex->addPoint(vertex);
			}

			btTransform offsetTrans(btMatrix3x3::getIdentity(), -info->massCenter);
			pCompound->addChildShape(offsetTrans, pConvex);
#endif
		}
	}

	return pCompound;
}

// Purpose: Loads and converts an ivp mesh to a bullet mesh.
void CPhysicsCollision::VCollideLoad(vcollide_t *pOutput, int solidCount, const char *pBuffer, int bufferSize, bool swap) {
	memset(pOutput, 0, sizeof(*pOutput));

	// TODO: This
	// In practice, this is never true on a Windows PC (and most likely not a linux dedicated server either)
	if (swap) {
		Warning("VCollideLoad - Abort loading, swap is true\n");
		Assert(0);
		return;
	}

	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide *[solidCount];

	int position = 0;
	for (int i = 0; i < solidCount; i++) {
		int size = *(int *)(pBuffer + position);
		position += 4; // Skip the size int.

		pOutput->solids[i] = (CPhysCollide *)(pBuffer + position);
		position += size;

		// May fail if we're reading a corrupted file.
		Assert(position < bufferSize);
	}

	// The rest of the buffer is the key values for the collision mesh
	pOutput->pKeyValues = new char[bufferSize - position];
	memcpy(pOutput->pKeyValues, pBuffer + position, bufferSize - position);

	// swap argument means byte swap - we must byte swap all of the collision shapes before loading them if true!
	DevMsg("VPhysics: VCollideLoad with %d solids, swap is %s\n", solidCount, swap ? "true" : "false");

	// Now for the fun part:
	// We must convert all of the ivp shapes into something we can use.
	for (int i = 0; i < solidCount; i++) {
		// NOTE: modelType 0 is IVPS, 1 is (mostly unused) MOPP format
		const compactsurfaceheader_t &surfaceheader = *(compactsurfaceheader_t *)pOutput->solids[i];

		if (surfaceheader.vphysicsID	!= VPHYSICS_ID
		 || surfaceheader.version		!= 0x100) {
			pOutput->solids[i] = NULL;
			continue;
		}

		btCollisionShape *pShape = NULL;
		if (surfaceheader.modelType == 0x0) {
			pShape = LoadIVPS(pOutput->solids[i], swap);
		} else if (surfaceheader.modelType == 0x1) {
			pShape = LoadMOPP(pOutput->solids[i], swap);
		} else {
			Warning("VCollideLoad: Unknown modelType %d", surfaceheader.modelType);
		}

		if (!pShape) {
			DevWarning("VCollideLoad: Failed to load a solid!\n");
		}

		pOutput->solids[i] = (CPhysCollide *)pShape;
	}
}

void CPhysicsCollision::VCollideUnload(vcollide_t *pVCollide) {
	for (int i = 0; i < pVCollide->solidCount; i++) {
		DestroyCollide(pVCollide->solids[i]);
	}

	delete [] pVCollide->solids;
	pVCollide->solids = NULL;

	delete [] pVCollide->pKeyValues;
	pVCollide->pKeyValues = NULL;
}

IVPhysicsKeyParser *CPhysicsCollision::VPhysicsKeyParserCreate(const char *pKeyData) {
	return new CPhysicsKeyParser(pKeyData);
}

void CPhysicsCollision::VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser) {
	delete (CPhysicsKeyParser *)pParser;
}

int CPhysicsCollision::CreateDebugMesh(CPhysCollide const *pCollisionModel, Vector **outVerts) {
	if (!pCollisionModel || !outVerts) return 0;

	btCollisionShape *pShape = (btCollisionShape *)pCollisionModel;
	int count = 0;

	if (pShape->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pShape;
		for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
			int shapeType = pCompound->getChildShape(i)->getShapeType();

			if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
				count += ((btConvexHullShape *)pCompound->getChildShape(i))->getNumVertices();
			} else if (shapeType == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
				count += ((btConvexTriangleMeshShape *)pCompound->getChildShape(i))->getNumVertices();
			}
		}
		
		if (count >= 0) {
			*outVerts = new Vector[count];
			int curVert = 0;

			for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
				int shapeType = pCompound->getChildShape(i)->getShapeType();

				if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
					btConvexHullShape *pConvex = (btConvexHullShape *)pCompound->getChildShape(i);

					// Source requires vertices in reverse order
					for (int j = pConvex->getNumVertices()-1; j >= 0; j--) {
						btVector3 pos;
						pConvex->getVertex(j, pos);
						ConvertPosToHL(pos, (*outVerts)[curVert++]);
					}
				} else if (shapeType == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
					// FYI: Currently unsupported in convex tri meshes
					btConvexTriangleMeshShape *pConvex = (btConvexTriangleMeshShape *)pCompound->getChildShape(i);

					for (int j = pConvex->getNumVertices()-1; j >= 0; j--) {
						btVector3 pos;
						pConvex->getVertex(j, pos);
						ConvertPosToHL(pos, (*outVerts)[curVert++]);
					}
				}
			}
		}
	}

	return count;
}

void CPhysicsCollision::DestroyDebugMesh(int vertCount, Vector *outVerts) {
	delete [] outVerts;
}

ICollisionQuery *CPhysicsCollision::CreateQueryModel(CPhysCollide *pCollide) {
	return new CCollisionQuery((btCollisionShape *)pCollide);
}

void CPhysicsCollision::DestroyQueryModel(ICollisionQuery *pQuery) {
	delete pQuery;
}

IPhysicsCollision *CPhysicsCollision::ThreadContextCreate() {
	return new CPhysicsCollision;
}

void CPhysicsCollision::ThreadContextDestroy(IPhysicsCollision *pThreadContext) {
	delete pThreadContext;
}

// BUG: Weird collisions with these, sometimes phys objs fall through the displacement mesh
CPhysCollide *CPhysicsCollision::CreateVirtualMesh(const virtualmeshparams_t &params) {
	IVirtualMeshEvent *pHandler = params.pMeshEventHandler;
	if (!pHandler) return NULL;

	// TODO: if params.buildOuterHull is true, what do we do?

	virtualmeshlist_t list;
	pHandler->GetVirtualMesh(params.userData, &list);

	btTriangleMesh *pMesh = new btTriangleMesh;

	btVector3 bullVec[3];
	for (int i = 0; i < list.triangleCount; i++) {
		ConvertPosToBull(list.pVerts[list.indices[i*3+0]], bullVec[0]);
		ConvertPosToBull(list.pVerts[list.indices[i*3+1]], bullVec[1]);
		ConvertPosToBull(list.pVerts[list.indices[i*3+2]], bullVec[2]);
		pMesh->addTriangle(bullVec[0], bullVec[1], bullVec[2], true);
	}

	btBvhTriangleMeshShape *bull = new btBvhTriangleMeshShape(pMesh, true);
	bull->setMargin(COLLISION_MARGIN);
	return (CPhysCollide *)bull;
}

bool CPhysicsCollision::SupportsVirtualMesh() {
	return true;
}

void CPhysicsCollision::OutputDebugInfo(const CPhysCollide *pCollide) {
	btCollisionShape *pShape = (btCollisionShape *)pCollide;

	Msg("Type: %s\n", pShape->getName());
	if (pShape->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pShape;
		Msg("Num child shapes: %d\n", pCompound->getNumChildShapes());

		Msg("-- CHILD SHAPES:\n");
		for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
			OutputDebugInfo((CPhysCollide *)pCompound->getChildShape(i));
		}
		Msg("---\n");
	} else if (pShape->isConvex()) {
		if (pShape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE) {
			btConvexHullShape *pConvex = (btConvexHullShape *)pShape;
			Msg("Margin: %f\n", pConvex->getMargin());
			Msg("Num points: %d\n", pConvex->getNumPoints());
		}
	}
}

unsigned int CPhysicsCollision::ReadStat(int statID) {
	NOT_IMPLEMENTED
	return 0;
}

CPhysicsCollision g_PhysicsCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, g_PhysicsCollision);
