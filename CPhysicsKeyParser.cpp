#include "StdAfx.h"

#include "CPhysicsKeyParser.h"

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
	return true;
}

void CPhysicsKeyParser::ParseSolid(solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler)
{
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pSolid);
	else
		memset(pSolid, 0, sizeof*pSolid);

	NextBlock();
}

void CPhysicsKeyParser::ParseFluid(fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler)
{
	NOT_IMPLEMENTED;
	NextBlock();
}

void CPhysicsKeyParser::ParseRagdollConstraint(constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler)
{
	NOT_IMPLEMENTED;
	NextBlock();
}

void CPhysicsKeyParser::ParseSurfaceTable(int *table, IVPhysicsKeyHandler *unknownKeyHandler)
{
	NOT_IMPLEMENTED;
	NextBlock();
}

void CPhysicsKeyParser::ParseCustom(void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler)
{
	NOT_IMPLEMENTED;
	NextBlock();
}

void CPhysicsKeyParser::ParseVehicle(vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler)
{
	NOT_IMPLEMENTED;
	NextBlock();
}
