#ifndef PHYSICS_VEHICLECONTROLLERCUSTOM_H
#define PHYSICS_VEHICLECONTROLLERCUSTOM_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

// TODO: Find a better include
#include <mathlib/vector.h>

// Class declarations

class CPhysicsEnvironment;
class CPhysicsObject;

class IPhysicsGameTrace;

// TODO: Move these classes to a public interface

// FIXME: Name conflict with public interface!
struct vehicle_wheelparams_t {
	// Position of wheel in world space
	Vector	worldPosition;

	// Normalized direction the wheel is facing (normally straight down, used for raytrace direction)
	Vector	wheelDirection;

	// Normalized direction of the axle from the wheel (normally straight left, used for which way the wheel rolls)
	Vector	wheelAxleDirection;

	// Radius of wheel in inches
	float	wheelRadius;

	// Is this a front wheel?
	bool	isFrontWheel;

	// Max suspension length in inches
	float	maxSuspensionLength;

	// How stiff the suspension is. Lower values make the vehicle sit on the ground, higher ones make the suspension hard (Usually some param * body mass)
	float	suspensionStiffness;

	inline void Defaults() {
		worldPosition.Zero();
		wheelDirection.Zero();
		wheelAxleDirection.Zero();

		wheelRadius			= 32;
		isFrontWheel		= true;
		maxSuspensionLength = 16;
		suspensionStiffness = 32;
	}
};

// Purpose: Structure to pass information about wheels back to the game
struct vehicle_wheelinfo_t {
	// Position of wheel in world space
	Vector	worldPosition;

	// Turn angle in degrees (-left/+right)
	float	turnAngle;
};

// Purpose: Custom wheel ray tracing
class IPhysicsVehicleWheelTrace {
	public:
		// Return the physics object the trace hit, or NULL
		virtual IPhysicsObject *CastRay(int wheelIndex, const Vector &start, const Vector &end, trace_t &result) = 0;
};

// Purpose: Brand new vehicle controller class for the purposes of allowing the game to implement
// their own vehicle types. (Too messy to add this functionality to the normal one)
class CPhysicsVehicleControllerCustom {
	public:
		CPhysicsVehicleControllerCustom(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, IPhysicsGameTrace *pGameTrace);
		~CPhysicsVehicleControllerCustom();

		// force in kg*in/s
		void								SetWheelForce(int wheelIndex, float force);
		void								SetWheelBrake(int wheelIndex, float brakeVal);
		// steerVal is in degrees!
		void								SetWheelSteering(int wheelIndex, float steerVal);

		void Update(float dt);

		void CreateWheel(const vehicle_wheelparams_t &params);

	public:
		// Unexposed functions

	private:
		CPhysicsEnvironment *	m_pEnv;
		CPhysicsObject *		m_pBody;
		IPhysicsGameTrace *		m_pGameTrace;
};

CPhysicsVehicleControllerCustom CreateVehicleControllerCustom(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, IPhysicsGameTrace *pGameTrace);

#endif // PHYSICS_VEHICLECONTROLLERCUSTOM_H