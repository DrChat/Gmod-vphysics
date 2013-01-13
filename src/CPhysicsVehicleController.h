#ifndef CPHYSICSVEHICLECONTROLLER_H
#define CPHYSICSVEHICLECONTROLLER_H

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
											CPhysicsVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace);
											~CPhysicsVehicleController();

		const vehicle_operatingparams_t &	GetOperatingParams() { return m_vehicleState; };
		const vehicleparams_t &				GetVehicleParams() { return m_vehicleParams; }
		vehicleparams_t &					GetVehicleParamsForChange() { return m_vehicleParams; }

		void								Update(float dt, vehicle_controlparams_t &controls);
		float								UpdateBooster(float dt);
		int									GetWheelCount(void);
		IPhysicsObject *					GetWheel(int index);
		bool								GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps);
		void								SetSpringLength(int wheelIndex, float length);
		void								SetWheelFriction(int wheelIndex, float friction);

		void								OnVehicleEnter(void) { m_bOccupied = true; }
		void								OnVehicleExit(void) { m_bOccupied = false; }

		void								SetEngineDisabled(bool bDisable) { m_bEngineDisabled = bDisable; }
		bool								IsEngineDisabled(void) { return m_bEngineDisabled; }

		// Debug
		void								GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem);
		void								VehicleDataReload() { NOT_IMPLEMENTED }

		// Unexposed functions
		void								InitVehicleParams(const vehicleparams_t &params);
		void								InitBullVehicle();

		void								InitCarWheels();
		CPhysicsObject *					CreateWheel(int wheelIndex, vehicle_axleparams_t &axle);

		void								UpdateSteering(const vehicle_controlparams_t &controls, float dt);
		void								UpdateEngine(const vehicle_controlparams_t &controls, float dt);
		void								UpdateWheels(const vehicle_controlparams_t &controls, float dt);

		void								CalcEngine(const vehicle_controlparams_t &controls, float dt);

		void								ShutdownBullVehicle();

	private:
		vehicleparams_t						m_vehicleParams;
		vehicle_operatingparams_t			m_vehicleState;
		CPhysicsObject *					m_pBody;
		CPhysicsEnvironment *				m_pEnv;
		IPhysicsGameTrace *					m_pGameTrace;
		unsigned int						m_iVehicleType;
		bool								m_bEngineDisabled;
		bool								m_bOccupied;
		CPhysicsObject *					m_pWheels[VEHICLE_MAX_WHEEL_COUNT];
		int									m_iWheelCount;
		bool								m_bSlipperyWheels;

		btVehicleRaycaster *				m_pRaycaster;
		btRaycastVehicle *					m_pRaycastVehicle;
		btRaycastVehicle::btVehicleTuning	m_tuning;
};

IPhysicsVehicleController *CreateVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace);

#endif // CPHYSICSVEHICLECONTROLLER_H