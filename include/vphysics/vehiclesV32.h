#ifndef VEHICLESV32_H
#define VEHICLESV32_h
#ifdef _MSC_VER
#pragma once
#endif

#include "vphysics/vehicles.h"

class IPhysicsVehicleController1 : public IPhysicsVehicleController {
	public:
		// Updates the internal vehicle without calculating anything related to controls.
		// Use this instead of Update if you want to manually control handling.
		// Also, you should be calling UpdateVehicle after you call all of the SetX functions
		virtual void					UpdateVehicle(float dt) = 0;

		// force in kg*in/s
		virtual void					SetWheelForce(int wheelIndex, float force) = 0;
		virtual void					SetWheelBrake(int wheelIndex, float brakeVal) = 0;
		// steerVal is in degrees!
		virtual void					SetWheelSteering(int wheelIndex, float steerVal) = 0;
};

#endif // VEHICLESV32_H