#ifndef CPHYSICSSURFACEPROPS_H
#define CPHYSICSSURFACEPROPS_H

enum {
	MATERIAL_INDEX_SHADOW = 0xF000,
};

class CSurface : public btMaterial {
public:
	CUtlSymbol m_name;
	surfacedata_t data;
};

class CPhysicsSurfaceProps : public IPhysicsSurfaceProps {
public:
	CPhysicsSurfaceProps();
	virtual ~CPhysicsSurfaceProps();
	virtual int ParseSurfaceData(const char* pFilename, const char* pTextfile);
	virtual int SurfacePropCount() const;

	virtual int GetSurfaceIndex(const char* pSurfacePropName) const;
	virtual void GetPhysicsProperties(int surfaceDataIndex, float* density, float* thickness, float* friction, float* elasticity) const;

	virtual surfacedata_t* GetSurfaceData(int surfaceDataIndex);
	virtual const char* GetString(unsigned short stringTableIndex) const;

	virtual const char* GetPropName(int surfaceDataIndex) const;

	virtual void SetWorldMaterialIndexTable(int* pMapArray, int mapSize);

	virtual void GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t* pParamsOut) const;
private:
	int GetReservedSurfaceIndex(const char* pSurfacePropName) const;

	CSurface* GetInternalSurface(int materialIndex);
	const CSurface* GetInternalSurface(int materialIndex) const;

	void CopyPhysicsProperties(CSurface *pOut, int baseIndex);
	bool AddFileToDatabase(const char *pFilename);
private:
	CUtlSymbolTable* m_strings;
	CUtlVector<CUtlSymbol> m_soundList;
	CUtlVector<CSurface> m_props;
	CUtlVector<CUtlSymbol> m_fileList;
};

#endif
