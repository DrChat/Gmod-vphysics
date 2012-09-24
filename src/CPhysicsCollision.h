#ifndef CPHYSICSCOLLISION_H
#define CPHYSICSCOLLISION_H

#include "CPhysicsTrace.h"

class CPhysicsCollision : public IPhysicsCollision {
	public:
		CPhysicsCollision();
		virtual ~CPhysicsCollision();

		virtual CPhysConvex* ConvexFromVerts(Vector **pVerts, int vertCount);
		virtual CPhysConvex* ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance);
		virtual float ConvexVolume(CPhysConvex *pConvex);
		virtual float ConvexSurfaceArea(CPhysConvex *pConvex);
		virtual void SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData);
		virtual void ConvexFree(CPhysConvex *pConvex);
		virtual CPhysConvex* BBoxToConvex(const Vector &mins, const Vector &maxs);
		virtual CPhysConvex* ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron);
		virtual void ConvexesFromConvexPolygon(const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput);

		virtual CPhysPolysoup* PolysoupCreate();
		virtual void PolysoupDestroy(CPhysPolysoup *pSoup);
		virtual void PolysoupAddTriangle(CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits);

		virtual CPhysCollide* ConvertPolysoupToCollide(CPhysPolysoup *pSoup, bool useMOPP);
		virtual CPhysCollide* ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount);
		virtual CPhysCollide* ConvertConvexToCollideParams(CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams);
		virtual void DestroyCollide(CPhysCollide *pCollide);
		virtual int CollideSize(CPhysCollide *pCollide);
		virtual int CollideWrite(char *pDest, CPhysCollide *pCollide, bool bSwap = false);
		virtual CPhysCollide* UnserializeCollide(char *pBuffer, int size, int index);
		virtual float CollideVolume(CPhysCollide *pCollide);
		virtual float CollideSurfaceArea(CPhysCollide *pCollide);
		virtual Vector CollideGetExtent(const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction);
		virtual void CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles);
		virtual void CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter);
		virtual void CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter);
		virtual Vector CollideGetOrthographicAreas(const CPhysCollide *pCollide);
		virtual void CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas);
		virtual int CollideIndex(const CPhysCollide *pCollide);
		virtual CPhysCollide* BBoxToCollide(const Vector &mins, const Vector &maxs);
		virtual int GetConvexesUsedInCollideable(const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit);

		virtual void TraceBox(const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		virtual void TraceBox(const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		virtual void TraceBox(const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		virtual void TraceCollide(const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);

		virtual bool IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone);

		virtual void VCollideLoad(vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap = false);
		virtual void VCollideUnload(vcollide_t *pVCollide);

		virtual IVPhysicsKeyParser* VPhysicsKeyParserCreate(const char *pKeyData);
		virtual void VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser);

		virtual int CreateDebugMesh(CPhysCollide const *pCollisionModel, Vector **outVerts);
		virtual void DestroyDebugMesh(int vertCount, Vector *outVerts);

		virtual ICollisionQuery* CreateQueryModel(CPhysCollide *pCollide);
		virtual void DestroyQueryModel(ICollisionQuery *pQuery);

		virtual IPhysicsCollision* ThreadContextCreate();
		virtual void ThreadContextDestroy(IPhysicsCollision *pThreadContex);

		virtual CPhysCollide* CreateVirtualMesh(const virtualmeshparams_t &params);
		virtual bool SupportsVirtualMesh();

		virtual bool GetBBoxCacheSize(int *pCachedSize, int *pCachedCount);

		virtual CPolyhedron* PolyhedronFromConvex(CPhysConvex * const pConvex, bool bUseTempPolyhedron);

		virtual void OutputDebugInfo(const CPhysCollide *pCollide);
		virtual unsigned int ReadStat(int statID);
	private:
		CPhysicsTrace		m_traceapi;
};

#endif
