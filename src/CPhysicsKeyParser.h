#ifndef CPHYSICSKEYPARSER_H
#define CPHYSICSKEYPARSER_H

class CPhysicsKeyParser : public IVPhysicsKeyParser
{
	public:
							CPhysicsKeyParser(const char *pKeyValues);
							~CPhysicsKeyParser();

		virtual void		NextBlock(void);

		virtual const char *GetCurrentBlockName(void);
		virtual bool		Finished(void);
		virtual void		ParseSolid(solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler);
		virtual void		ParseFluid(fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler);
		virtual void		ParseRagdollConstraint(constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler);
		virtual void		ParseSurfaceTable(int *table, IVPhysicsKeyHandler *unknownKeyHandler);
		virtual void		ParseCustom(void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler);
		virtual void		ParseVehicle(vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler);
		virtual void		SkipBlock(void) { NextBlock(); };

		virtual void		ParseVehicleAxle(vehicle_axleparams_t &suspension, KeyValues *kv);
		virtual void		ParseVehicleBody(vehicle_bodyparams_t &body, KeyValues *kv);
		virtual void		ParseVehicleEngine(vehicle_engineparams_t &engine, KeyValues *kv);
		virtual void		ParseVehicleEngineBoost(vehicle_engineparams_t &engine, KeyValues *kv);
		virtual void		ParseVehicleSteering(vehicle_steeringparams_t &steering, KeyValues *kv);

	private:
		KeyValues *			m_pKeyValues;
		KeyValues *			m_pCurrentBlock;
};

#endif