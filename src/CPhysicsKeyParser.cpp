#include "StdAfx.h"

#include "CPhysicsKeyParser.h"
#include "CPhysicsSurfaceProps.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

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
		return m_pCurrentBlock->GetName();
	}
	return NULL;
}

bool CPhysicsKeyParser::Finished(void)
{
	return m_pCurrentBlock == NULL;
}

void CPhysicsKeyParser::ParseSolid(solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler)
{
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pSolid);
	else
		memset(pSolid, 0, sizeof*pSolid);

	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();
		if (!stricmp(key, "index"))
			pSolid->index = data->GetInt();
		else if (!stricmp(key, "surfaceprop"))
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

	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();
		if (!stricmp(key, "index"))
			pFluid->index = data->GetInt();
		else if (!stricmp(key, "surfaceprop"))
			strncpy(pFluid->surfaceprop, data->GetString(), sizeof pFluid->surfaceprop);
		else if (!stricmp(key, "contents"))
			pFluid->params.contents = data->GetInt();
		//if (!stricmp(key, "density")) // In the 2003 leak this existed, in the current code pFluid->params.density does not
		else if (!stricmp(key, "damping"))
			pFluid->params.damping = data->GetFloat();
		else if (!stricmp(key, "surfaceplane"))
			ReadVector4D(data->GetString(), pFluid->params.surfacePlane);
		else if (!stricmp(key, "currentvelocity"))
			ReadVector(data->GetString(), pFluid->params.currentVelocity);
		else if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pFluid, key, data->GetString());
	}
	NextBlock();
}

void CPhysicsKeyParser::ParseRagdollConstraint(constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler)
{
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pConstraint);
	else
	{
		memset(pConstraint, 0, sizeof*pConstraint);
		pConstraint->childIndex = -1;
		pConstraint->parentIndex = -1;
	}

	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();
		if (!stricmp(key, "parent"))
			pConstraint->parentIndex = data->GetInt();
		if (!stricmp(key, "child"))
			pConstraint->childIndex = data->GetInt();
		else if (!stricmp(key, "xmin"))
			pConstraint->axes[0].minRotation = data->GetFloat();
		else if (!stricmp(key, "xmax"))
			pConstraint->axes[0].maxRotation = data->GetFloat();
		else if (!stricmp(key, "xfriction"))
		{
			pConstraint->axes[0].angularVelocity = 0;
			pConstraint->axes[0].torque = data->GetFloat();
		}
		else if (!stricmp(key, "ymin"))
			pConstraint->axes[1].minRotation = data->GetFloat();
		else if (!stricmp(key, "ymax"))
			pConstraint->axes[1].maxRotation = data->GetFloat();
		else if (!stricmp(key, "yfriction"))
		{
			pConstraint->axes[1].angularVelocity = 0;
			pConstraint->axes[1].torque = data->GetFloat();
		}
		else if (!stricmp(key, "zmin"))
			pConstraint->axes[2].minRotation = data->GetFloat();
		else if (!stricmp(key, "zmax"))
			pConstraint->axes[2].maxRotation = data->GetFloat();
		else if (!stricmp(key, "zfriction"))
		{
			pConstraint->axes[2].angularVelocity = 0;
			pConstraint->axes[2].torque = data->GetFloat();
		}
		else if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pConstraint, key, data->GetString());
	}
	NextBlock();
}

void CPhysicsKeyParser::ParseSurfaceTable(int *table, IVPhysicsKeyHandler *unknownKeyHandler)
{
	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		if (data->GetInt() < 128)
			table[data->GetInt()] = g_SurfaceDatabase.GetSurfaceIndex(data->GetName());
	}
	NextBlock();
}

void CPhysicsKeyParser::ParseCustom(void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler)
{
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pCustom);
	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();
		const char *value = data->GetString();
		if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pCustom, key, value);
	}
	NextBlock();
}

void CPhysicsKeyParser::ParseVehicle(vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler)
{
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pVehicle);
	else
		memset(pVehicle, 0, sizeof*pVehicle);

	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!stricmp(key, "axle") && pVehicle->axleCount < VEHICLE_MAX_AXLE_COUNT)
			ParseVehicleAxle(pVehicle->axles[pVehicle->axleCount++], data);
		else if (!stricmp(key, "body"))
			ParseVehicleBody(pVehicle->body, data);
		else if (!stricmp(key, "engine"))
			ParseVehicleEngine(pVehicle->engine, data);
		else if (!stricmp(key, "steering"))
			ParseVehicleSteering(pVehicle->steering, data);
		else if (!stricmp(key, "wheelsperaxle"))
			pVehicle->wheelsPerAxle = data->GetInt();
	}
	NextBlock();
}


void CPhysicsKeyParser::ParseVehicleAxle(vehicle_axleparams_t &axle, KeyValues *kv)
{
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!stricmp(key, "wheel"))
			ParseVehicleWheel(axle.wheels, data);
		else if (!stricmp(key, "suspension"))
			ParseVehicleSuspension(axle.suspension, data);
		else if (!stricmp(key, "torquefactor"))
			axle.torqueFactor = data->GetFloat();
		else if (!stricmp(key, "brakefactor"))
			axle.brakeFactor = data->GetFloat();
	}
}

void CPhysicsKeyParser::ParseVehicleWheel(vehicle_wheelparams_t &wheel, KeyValues *kv)
{
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!stricmp(key, "radius"))
			wheel.radius = data->GetFloat();
		else if (!stricmp(key, "mass"))
			wheel.mass = data->GetFloat();
		else if (!stricmp(key, "damping"))
			wheel.damping = data->GetFloat();
		else if (!stricmp(key, "rotdamping"))
			wheel.rotdamping = data->GetFloat();
		else if (!stricmp(key, "material"))
			wheel.materialIndex = data->GetInt();		// TODO: Is this correct?
		else if (!stricmp(key, "skidmaterial"))
			wheel.skidMaterialIndex = data->GetInt();	// TODO: Is this correct?
		else if (!stricmp(key, "brakematerial"))
			wheel.brakeMaterialIndex = data->GetInt();	// TODO: Is this correct?
	}
}

void CPhysicsKeyParser::ParseVehicleSuspension(vehicle_suspensionparams_t &suspension, KeyValues *kv)
{
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!stricmp(key, "springConstant"))
			suspension.springConstant = data->GetFloat();
		else if (!stricmp(key, "springDamping"))
			suspension.springDamping = data->GetFloat();
		else if (!stricmp(key, "stabilizerConstant"))
			suspension.stabilizerConstant = data->GetFloat();
		else if (!stricmp(key, "springDampingCompression"))
			suspension.springDampingCompression = data->GetFloat();
		else if (!stricmp(key, "maxBodyForce"))
			suspension.maxBodyForce = data->GetFloat();
	}
}

void CPhysicsKeyParser::ParseVehicleBody(vehicle_bodyparams_t &body, KeyValues *kv)
{
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!stricmp(key, "massCenterOverride"))
			ReadVector(data->GetString(), body.massCenterOverride);
		else if (!stricmp(key, "addgravity"))
			body.addGravity = data->GetFloat();
		else if (!stricmp(key, "massOverride"))
			body.massOverride = data->GetFloat();
		else if (!stricmp(key, "tiltforce"))
			body.tiltForce = data->GetFloat();
		else if (!stricmp(key, "tiltforceheight"))
			body.tiltForceHeight = data->GetFloat();
		else if (!stricmp(key, "countertorquefactor"))
			body.counterTorqueFactor = data->GetFloat();
		else if (!stricmp(key, "keepuprighttorque"))
			body.keepUprightTorque = data->GetFloat();
	}
}

void CPhysicsKeyParser::ParseVehicleEngine(vehicle_engineparams_t &engine, KeyValues *kv)
{
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();
		if (!stricmp(key, "boost"))
			ParseVehicleEngineBoost(engine, data);
		else if (!stricmp(key, "gear") && engine.gearCount < VEHICLE_MAX_GEAR_COUNT)
			engine.gearRatio[engine.gearCount++] = data->GetFloat();
		else if (!stricmp(key, "horsepower"))
			engine.horsepower = data->GetFloat();
		else if (!stricmp(key, "maxSpeed"))
			engine.maxSpeed = data->GetFloat();
		else if (!stricmp(key, "maxReverseSpeed"))
			engine.maxRevSpeed = data->GetFloat();
		else if (!stricmp(key, "axleratio"))
			engine.axleRatio = data->GetFloat();
		else if (!stricmp(key, "maxRPM"))
			engine.maxRPM = data->GetFloat();
		else if (!stricmp(key, "throttleTime"))
			engine.throttleTime = data->GetFloat();
		else if (!stricmp(key, "AutoTransmission"))
			engine.isAutoTransmission = data->GetInt() > 0;
		else if (!stricmp(key, "shiftUpRPM"))
			engine.shiftUpRPM = data->GetFloat();
		else if (!stricmp(key, "shiftDownRPM"))
			engine.shiftDownRPM = data->GetFloat();
	}
}

void CPhysicsKeyParser::ParseVehicleEngineBoost(vehicle_engineparams_t &engine, KeyValues *kv)
{
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();
		if (!stricmp(key, "force"))
			engine.boostForce = data->GetFloat();
		else if (!stricmp(key, "duration"))
			engine.boostDuration = data->GetFloat();
		else if (!stricmp(key, "delay"))
			engine.boostDelay = data->GetFloat();
		else if (!stricmp(key, "maxspeed"))
			engine.boostMaxSpeed = data->GetFloat();
		else if (!stricmp(key, "torqueboost"))
			engine.torqueBoost = data->GetInt() > 0;
	}
}

void CPhysicsKeyParser::ParseVehicleSteering(vehicle_steeringparams_t &steering, KeyValues *kv)
{
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();
		if (!stricmp(key, "degrees")) // FIXME: Bit unsure here, in the 2003 code it just set "degrees" which does not exist anymore
		{
			steering.degreesBoost = data->GetFloat();
			steering.degreesFast = data->GetFloat();
			steering.degreesSlow = data->GetFloat();
		}
		if (!stricmp(key, "fastcarspeed"))
			steering.speedFast = data->GetFloat();
		if (!stricmp(key, "slowcarspeed"))
			steering.speedSlow = data->GetFloat();
		else if (!stricmp(key, "slowsteeringrate"))
			steering.steeringRateSlow = data->GetFloat();
		else if (!stricmp(key, "faststeeringrate"))
			steering.steeringRateFast = data->GetFloat();
		//else if (!stricmp(key, "steeringRestFactor")) // This one was in the 2003 leak, its used in some vehicle scripts but I dont know which variable it should set
		else if (!stricmp(key, "skidallowed"))
			steering.isSkidAllowed = data->GetInt() > 0;
		else if (!stricmp(key, "dustcloud"))
			steering.dustCloud = data->GetInt() > 0;
	}
}
