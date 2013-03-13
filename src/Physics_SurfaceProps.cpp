#include "StdAfx.h"

#include <tier1/KeyValues.h>

#include "Physics_SurfaceProps.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/*****************************
* CLASS CPhysicsSurfaceProps
*****************************/

CPhysicsSurfaceProps::CPhysicsSurfaceProps() {
	m_strings = new CUtlSymbolTable(0, 32, true);
}

CPhysicsSurfaceProps::~CPhysicsSurfaceProps() {
	delete m_strings;
}

int CPhysicsSurfaceProps::ParseSurfaceData(const char *pFilename, const char *pTextfile) {
	if (!AddFileToDatabase(pFilename)) return 0;
	DevMsg("VPhysics: Parsing surface data (file: %s)\n", pFilename);

	KeyValues *surfprops = new KeyValues("CPhysicsSurfaceProps");
	surfprops->LoadFromBuffer(pFilename, pTextfile);
	for (KeyValues *surface = surfprops; surface; surface = surface->GetNextKey()) {
		CSurface prop;
		int baseMaterial = GetSurfaceIndex("default");

		memset(&prop.data, 0, sizeof(prop.data));
		prop.m_name = m_strings->AddString(surface->GetName());
		prop.data.game.material = 0;
		prop.data.game.maxSpeedFactor = 1.0f;
		prop.data.game.jumpFactor = 1.0f;
		prop.data.game.climbable = 0.0f;
		CopyPhysicsProperties(&prop, baseMaterial);

		for (KeyValues *data = surface->GetFirstSubKey(); data; data = data->GetNextKey()) {
			const char *key = data->GetName();
			if (!Q_stricmp(key, "base")) {
				baseMaterial = GetSurfaceIndex(data->GetString());
				CopyPhysicsProperties(&prop, baseMaterial);
			} else if (!Q_stricmp(key, "thickness"))
				prop.data.physics.thickness = data->GetFloat();
			else if (!Q_stricmp(key, "density"))
				prop.data.physics.density = data->GetFloat();
			else if (!Q_stricmp(key, "elasticity"))
				prop.data.physics.elasticity = data->GetFloat();
			else if (!Q_stricmp(key, "friction"))
				prop.data.physics.friction = data->GetFloat();
			else if (!Q_stricmp(key, "dampening"))
				prop.data.physics.dampening = data->GetFloat();
			else if (!Q_stricmp(key, "audioreflectivity"))
				prop.data.audio.reflectivity = data->GetFloat();
			else if (!Q_stricmp(key, "audiohardnessfactor"))
				prop.data.audio.hardnessFactor = data->GetFloat();
			else if (!Q_stricmp(key, "audioroughnessfactor"))
				prop.data.audio.roughnessFactor = data->GetFloat();
			else if (!Q_stricmp(key, "scrapeRoughThreshold"))
				prop.data.audio.roughThreshold = data->GetFloat();
			else if (!Q_stricmp(key, "impactHardThreshold"))
				prop.data.audio.hardThreshold = data->GetFloat();
			else if (!Q_stricmp(key, "audioHardMinVelocity"))
				prop.data.audio.hardVelocityThreshold = data->GetFloat();
			else if (!Q_stricmp(key, "maxspeedfactor"))
				prop.data.game.maxSpeedFactor = data->GetFloat();
			else if (!Q_stricmp(key, "jumpfactor"))
				prop.data.game.jumpFactor = data->GetFloat();
			else if (!Q_stricmp(key, "climbable"))
				prop.data.game.climbable = data->GetInt();
			else if (!Q_stricmp(key, "gamematerial"))
				if (data->GetDataType() == KeyValues::TYPE_STRING && strlen(data->GetString()) == 1) {
					prop.data.game.material = toupper(data->GetString()[0]);
				} else {
					prop.data.game.material = data->GetInt();
				}
			else if (!Q_stricmp(key, "stepleft")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.stepleft = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "stepright")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.stepright = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "impactsoft")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.impactSoft = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "impacthard")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.impactHard = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "scrapesmooth")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.scrapeSmooth = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "scraperough")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.scrapeRough = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "bulletimpact")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.bulletImpact = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "break")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.breakSound = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "strain")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.strainSound = m_soundList.AddToTail(sym);
			} else if (!Q_stricmp(key, "rolling")) {
				CUtlSymbol sym = m_strings->AddString(data->GetString());
				prop.data.sounds.rolling = m_soundList.AddToTail(sym);
			} else
				AssertMsg2(0, "Bad surfaceprop key %s (%s)\n", key, data->GetString());
		}
		if (GetSurfaceIndex(m_strings->String(prop.m_name)) >= 0) break;

		m_props.AddToTail(prop);
	}
	surfprops->deleteThis();
	return 0;
}

int CPhysicsSurfaceProps::SurfacePropCount() const {
	return m_props.Size();
}

int CPhysicsSurfaceProps::GetSurfaceIndex(const char *pSurfacePropName) const {
	if (pSurfacePropName[0] == '$') {
		int index = GetReservedSurfaceIndex(pSurfacePropName);
		if (index >= 0) return index;
	}

	CUtlSymbol id = m_strings->Find(pSurfacePropName);
	if (id.IsValid()) {
		for (int i = 0; i < m_props.Size(); i++) {
			if (m_props[i].m_name == id) return i;
		}
	}

	return -1;
}

void CPhysicsSurfaceProps::GetPhysicsProperties(int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity) const {
	const CSurface *pSurface = GetInternalSurface(surfaceDataIndex);
	if (pSurface) {
		if (friction) *friction		= pSurface->data.physics.friction;
		if (elasticity) *elasticity = pSurface->data.physics.elasticity;
		if (density) *density		= pSurface->data.physics.density;
		if (thickness) *thickness	= pSurface->data.physics.thickness;
	}
}

surfacedata_t *CPhysicsSurfaceProps::GetSurfaceData(int surfaceDataIndex) {
	CSurface *pSurface = GetInternalSurface(surfaceDataIndex);
	if (!pSurface) pSurface = GetInternalSurface(GetSurfaceIndex("default"));
	assert(pSurface);

	return &pSurface->data;
}

const char *CPhysicsSurfaceProps::GetString(unsigned short stringTableIndex) const {
	CUtlSymbol index = m_soundList[stringTableIndex];
	return m_strings->String(index);
}

const char *CPhysicsSurfaceProps::GetPropName(int surfaceDataIndex) const {
	if (surfaceDataIndex < 0 || surfaceDataIndex > m_props.Size())
		return "default";

	return m_strings->String(m_props[surfaceDataIndex].m_name);
}

void CPhysicsSurfaceProps::SetWorldMaterialIndexTable(int *pMapArray, int mapSize) {
	NOT_IMPLEMENTED
}

void CPhysicsSurfaceProps::GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t *pParamsOut) const {
	if (!pParamsOut) return;

	const CSurface *pSurface = GetInternalSurface(surfaceDataIndex);
	if (pSurface) {
		*pParamsOut = pSurface->data.physics;
	}
}

int CPhysicsSurfaceProps::GetReservedSurfaceIndex(const char *pSurfacePropName) const {
	if (!Q_stricmp(pSurfacePropName, "$MATERIAL_INDEX_SHADOW"))
		return MATERIAL_INDEX_SHADOW;
	
	return -1;
}

CSurface *CPhysicsSurfaceProps::GetInternalSurface(int materialIndex) {
	if (materialIndex < 0 || materialIndex > m_props.Size()-1)
		return NULL;

	return &m_props[materialIndex];
}

const CSurface *CPhysicsSurfaceProps::GetInternalSurface(int materialIndex) const {
	if (materialIndex < 0 || materialIndex > m_props.Size()-1)
		return NULL;

	return &m_props[materialIndex];
}

void CPhysicsSurfaceProps::CopyPhysicsProperties(CSurface *pOut, int baseIndex) {
	const CSurface *pSurface = GetInternalSurface(baseIndex);
	if (pSurface) {
		pOut->data.physics.density = pSurface->data.physics.density;
		pOut->data.physics.thickness = pSurface->data.physics.thickness;
		pOut->data.physics.friction = pSurface->data.physics.friction;
		pOut->data.physics.elasticity = pSurface->data.physics.elasticity;

		pOut->data.game.material = pSurface->data.game.material;
	}
}

bool CPhysicsSurfaceProps::AddFileToDatabase(const char *pFilename) {
	CUtlSymbol id = m_strings->AddString(pFilename);

	for (int i = 0; i < m_fileList.Size(); i++)
		if (m_fileList[i] == id) return false;

	m_fileList.AddToTail(id);
	return true;
}

CPhysicsSurfaceProps g_SurfaceDatabase;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsSurfaceProps, IPhysicsSurfaceProps, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION, g_SurfaceDatabase);
