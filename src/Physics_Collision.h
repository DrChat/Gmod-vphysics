#ifndef CPHYSICSCOLLISION_H
#define CPHYSICSCOLLISION_H

struct PhysicsShapeInfo {
	btVector3 massCenter;
};

// NOTE: There can only be up to 15 unique collision groups!
enum ECollisionGroups {
	COLGROUP_NONE	= 0,
	COLGROUP_WORLD	= 1<<1,
};

class CPhysicsCollision : public IPhysicsCollision1 {
	public:
		CPhysConvex *			ConvexFromVerts(Vector **pVerts, int vertCount);
		CPhysConvex *			ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance);
		float					ConvexVolume(CPhysConvex *pConvex);
		float					ConvexSurfaceArea(CPhysConvex *pConvex);
		void					SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData);
		void					ConvexFree(CPhysConvex *pConvex);
		CPhysConvex *			ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron);
		void					ConvexesFromConvexPolygon(const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput);

		CPhysPolysoup *			PolysoupCreate();
		void					PolysoupDestroy(CPhysPolysoup *pSoup);
		void					PolysoupAddTriangle(CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits);

		CPhysCollide *			ConvertPolysoupToCollide(CPhysPolysoup *pSoup, bool useMOPP);
		CPhysCollide *			ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount);
		CPhysCollide *			ConvertConvexToCollideParams(CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams);
		void					DestroyCollide(CPhysCollide *pCollide);
		int						CollideSize(CPhysCollide *pCollide);
		int						CollideWrite(char *pDest, CPhysCollide *pCollide, bool bSwap = false);
		CPhysCollide *			UnserializeCollide(char *pBuffer, int size, int index);
		float					CollideVolume(CPhysCollide *pCollide);
		float					CollideSurfaceArea(CPhysCollide *pCollide);
		Vector					CollideGetExtent(const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction);
		void					CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles);
		void					CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter);
		void					CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter);
		Vector					CollideGetOrthographicAreas(const CPhysCollide *pCollide);
		void					CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas);
		int						CollideIndex(const CPhysCollide *pCollide);
		int						GetConvexesUsedInCollideable(const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit);

		CPhysConvex *			BBoxToConvex(const Vector &mins, const Vector &maxs);
		CPhysCollide *			BBoxToCollide(const Vector &mins, const Vector &maxs);

		CPhysConvex *			CylinderToConvex(const Vector &mins, const Vector &maxs);
		CPhysCollide *			CylinderToCollide(const Vector &mins, const Vector &maxs);

		CPhysConvex *			ConeToConvex(const float radius, const float height);
		CPhysCollide *			ConeToCollide(const float radius, const float height);

		CPhysConvex *			SphereToConvex(const float radius);
		CPhysCollide *			SphereToCollide(const float radius);

		void					TraceBox(const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		void					TraceBox(const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		void					TraceBox(const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		void					TraceCollide(const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);

		bool					IsBoxIntersectingCone(const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone);

		void					VCollideLoad(vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap = false);
		void					VCollideUnload(vcollide_t *pVCollide);

		IVPhysicsKeyParser *	VPhysicsKeyParserCreate(const char *pKeyData);
		void					VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser);

		int						CreateDebugMesh(CPhysCollide const *pCollisionModel, Vector **outVerts);
		void					DestroyDebugMesh(int vertCount, Vector *outVerts);

		ICollisionQuery *		CreateQueryModel(CPhysCollide *pCollide);
		void					DestroyQueryModel(ICollisionQuery *pQuery);

		IPhysicsCollision *		ThreadContextCreate();
		void					ThreadContextDestroy(IPhysicsCollision *pThreadContex);

		CPhysCollide *			CreateVirtualMesh(const virtualmeshparams_t &params);
		bool					SupportsVirtualMesh();

		bool					GetBBoxCacheSize(int *pCachedSize, int *pCachedCount);

		CPolyhedron *			PolyhedronFromConvex(CPhysConvex *const pConvex, bool bUseTempPolyhedron);

		void					OutputDebugInfo(const CPhysCollide *pCollide);
		unsigned int			ReadStat(int statID);
};

extern CPhysicsCollision g_PhysicsCollision;

#endif
