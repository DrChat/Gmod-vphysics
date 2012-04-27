class IPhysicsVehicleController;
class IPhysicsObject;
class CPhysicsEnvironment;
struct vehicleparams_t;
class Vector;
struct vehicle_operatingparams_t;
struct vehicle_controlparams_t;
struct vehicle_debugcarsystem_t;

class CPhysicsVehicleController : public IPhysicsVehicleController
{
public:
	CPhysicsVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType);
	~CPhysicsVehicleController();

	virtual void Update(float dt, vehicle_controlparams_t &controls);
	virtual const vehicle_operatingparams_t &GetOperatingParams() { return m_vehicleState; };
	virtual const vehicleparams_t &GetVehicleParams() { return m_vehicleParams; }
	virtual vehicleparams_t &GetVehicleParamsForChange() { return m_vehicleParams; }
	virtual float UpdateBooster(float dt);
	virtual int GetWheelCount(void);
	virtual IPhysicsObject *GetWheel(int index);
	virtual bool GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps);
	virtual void SetSpringLength(int wheelIndex, float length);
	virtual void SetWheelFriction(int wheelIndex, float friction);

	virtual void OnVehicleEnter(void);
	virtual void OnVehicleExit(void);

	virtual void SetEngineDisabled(bool bDisable) { m_bEngineDisabled = bDisable; }
	virtual bool IsEngineDisabled(void) { return m_bEngineDisabled; }

	// Debug
	virtual void GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem) { NOT_IMPLEMENTED; }
	virtual void VehicleDataReload() { NOT_IMPLEMENTED; }

private:
	vehicleparams_t m_vehicleParams;
	vehicle_operatingparams_t m_vehicleState;
	CPhysicsObject *m_pBody;
	CPhysicsEnvironment *m_pEnv;
	unsigned int m_iVehicleType;
	bool m_bEngineDisabled;

	btVehicleRaycaster *m_pRaycaster;
	btRaycastVehicle *m_pRaycastVehicle;
	btRaycastVehicle::btVehicleTuning m_tuning;
};