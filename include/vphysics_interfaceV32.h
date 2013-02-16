// Purpose: Updated interface to vphysics.dll
// This interface MUST be ADDITIVE ONLY. DO NOT change old function signatures.

#ifndef VPHYSICS_INTERFACEV32_H
#define VPHYSICS_INTERFACEV32_H
#ifdef _MSC_VER
#pragma once
#endif

#include "tier1/interface.h"
#include "vphysics_interface.h"

abstract_class IPhysics1 : public IPhysics {
	public:
		virtual int		GetNumActiveEnvironments() = 0;
};

// TODO: Soft bodies
abstract_class IPhysicsEnvironment1 : public IPhysicsEnvironment {
	public:
};

abstract_class IPhysicsObject1 : public IPhysicsObject {
	public:
		virtual void	SetGravity(const Vector &gravityVector) = 0;
		virtual Vector	GetGravity() const = 0;
};

abstract_class IPhysicsCollision1 : public IPhysicsCollision {
	public:
		virtual CPhysConvex *	CylinderToConvex(const Vector &mins, const Vector &maxs) = 0;
		virtual CPhysCollide *	CylinderToCollide(const Vector &mins, const Vector &maxs) = 0;
};

#endif // VPHYSICS_INTERFACEV32_H