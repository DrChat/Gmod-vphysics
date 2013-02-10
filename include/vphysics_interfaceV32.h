// Purpose: Updated interface to vphysics.dll
// This interface MUST be additive only. DO NOT change old function signatures.

#ifndef VPHYSICS_INTERFACEV32_H
#define VPHYSICS_INTERFACEV32_H
#ifdef _MSC_VER
#pragma once
#endif

#include "tier1/interface.h"
#include "vphysics_interface.h"

// TODO: Soft bodies
abstract_class IPhysicsEnvironment1 : public IPhysicsEnvironment {
	public:
};

abstract_class IPhysicsObject1 : public IPhysicsObject {
	public:
		virtual void		SetGravity(const Vector &gravityVector) = 0;
		virtual Vector		GetGravity() const = 0;
};

#endif // VPHYSICS_INTERFACEV32_H