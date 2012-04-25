#include "StdAfx.h"

#include "CPhysicsKeyParser.h"
#include "CPhysicsSurfaceProps.h"

extern CPhysicsSurfaceProps g_SurfaceDatabase;

static void ReadVector(const char *pString, Vector& out)
{
	float x, y, z;
	sscanf(pString, "%f %f %f", &x, &y, &z);
	out.x = x;
	out.y = y;
	out.z = z;
}

static void ReadVector4D(const char *pString, Vector4D& out)
{
	float x, y, z, w;
	sscanf(pString, "%f %f %f %f", &x, &y, &z, &w);
	out.x = x;
	out.y = y;
	out.z = z;
	out.w = w;
}

CPhysicsKeyParser::CPhysicsKeyParser(const char *pKeyValues)
{
	m_pKeyValues = new KeyValues("CPhysicsKeyParser");
	m_pKeyValues->LoadFromBuffer("CPhysicsKeyParser", pKeyValues);
	m_pCurrentBlock = m_pKeyValues;
}

CPhysicsKeyParser::~CPhysicsKeyParser()
{
	if (m_pKeyValues)
		m_pKeyValues->deleteThis();
}

void CPhysicsKeyParser::NextBlock(void)
{
	if (m_pCurrentBlock)
		m_pCurrentBlock = m_pCurrentBlock->GetNextKey();
}

const char *CPhysicsKeyParser::GetCurrentBlockName(void)
{
	if (m_pCurrentBlock)
	{
		strcpy_s(m_pCurrentBlockName, m_pCurrentBlock->GetName());
		return m_pCurrentBlockName;
	}
	return NULL;
}

bool CPhysicsKeyParser::Finished(void)
{
	return m_pCurrentBlockName == NULL;
}

void CPhysicsKeyParser::ParseSolid(solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler)
{
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pSolid);
	else
		memset(pSolid, 0, sizeof*pSolid);

	for (KeyValues* data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char* key = data->GetName();
		if (!stricmp(key, "index"))
			pSolid->index = data->GetInt();
		if (!stricmp(key, "surfaceprop"))
			strncpy(pSolid->surfaceprop, data->GetString(), sizeof pSolid->surfaceprop);
		else if (!stricmp(key, "name"))
			strncpy(pSolid->name, data->GetString(), sizeof pSolid->name);
		else if (!stricmp(key, "parent"))
			strncpy(pSolid->parent, data->GetString(), sizeof pSolid->name);
		else if (!stricmp(key, "surfaceprop"))
			strncpy(pSolid->surfaceprop, data->GetString(), sizeof pSolid->name);
		else if (!stricmp(key, "mass"))
			pSolid->params.mass = data->GetFloat();
		else if (!stricmp(key, "massCenterOverride"))
			ReadVector(data->GetString(), pSolid->massCenterOverride);
		else if (!stricmp(key, "inertia"))
			pSolid->params.inertia = data->GetFloat();
		else if (!stricmp(key, "damping"))
			pSolid->params.damping = data->GetFloat();
		else if (!stricmp(key, "rotdamping"))
			pSolid->params.rotdamping = data->GetFloat();
		else if (!stricmp(key, "volume"))
			pSolid->params.volume = data->GetFloat();
		else if (!stricmp(key, "drag"))
			pSolid->params.dragCoefficient = data->GetFloat();
		//else if (!stricmp(key, "rollingdrag")) // This goes to pSolid->params.rollingDrag in the 2003 source but I cant find it in here
		else if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pSolid, key, data->GetString());
	}
	NextBlock();
}

void CPhysicsKeyParser::ParseFluid(fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler)
{
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pFluid);
	else
		memset(pFluid, 0, sizeof*pFluid);

	for (KeyValues* data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char* key = data->GetName();
		if (!stricmp(key, "index"))
			pFluid->index = data->GetInt();
		if (!stricmp(key, "surfaceprop"))
			strncpy(pFluid->surfaceprop, data->GetString(), sizeof pFluid->surfaceprop);
		if (!stricmp(key, "conents"))
			pFluid->params.contents = data->GetInt();
		//if (!stricmp(key, "density")) // In the 2003 leak this existed, in the current code pFluid->params.density does not
		if (!stricmp(key, "damping"))
			pFluid->params.damping = data->GetFloat();
		if (!stricmp(key, "surfaceplane"))
			pFluid->params.damping = data->GetFloat();
		if (!stricmp(key, "currentvelocity"))
			ReadVector4D(data->GetString(), pFluid->params.surfacePlane);
		if (!stricmp(key, "currentvelocity"))
			ReadVector(data->GetString(), pFluid->params.currentVelocity);
		else if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pFluid, key, data->GetString());
	}
	NextBlock();
}

void CPhysicsKeyParser::ParseRagdollConstraint(constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler)
{
	Error("shit1");
	NextBlock();
}

void CPhysicsKeyParser::ParseSurfaceTable(int *table, IVPhysicsKeyHandler *unknownKeyHandler)
{
	for (KeyValues* data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		if (data->GetInt() < 128)
			table[data->GetInt()] = g_SurfaceDatabase.GetSurfaceIndex(data->GetName());
	}
	NextBlock();
}

void CPhysicsKeyParser::ParseCustom(void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler)
{
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pCustom);
	for (KeyValues* data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char* key = data->GetName();
		const char* value = data->GetString();
		if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pCustom, key, value);
	}
	NextBlock();
}

void CPhysicsKeyParser::ParseVehicle(vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler)
{
	Error("shit2");
	NextBlock();
}
