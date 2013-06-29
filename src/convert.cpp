#include "StdAfx.h"

#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/************************************************
* COORDINATE SYSTEMS:
* Bullet vector: Forward, Up, Right
*	+x: forward (east)
*	+y: up
*	+z: right (south)
*
*
*
* HL Vector: Forward, Left, Up
*	+x: forward (east)
*	+y: left (north)
*	+z: up
*
*   (top down)
*
*   left (y)
*    ^
*    |
*   (*)------> forward (x)
*   up (z)
*
************************************************/

#ifdef _MSC_VER
	// Conversion from x to x, possible loss of data
	#pragma warning(disable: 4244)
#endif

// IVP: Forward down left
// IVP Units in meters
void ConvertIVPPosToBull(const float *pos, btVector3 &bull) {
	if (!pos) return;

	bull.setX(pos[0]);
	bull.setY(-pos[1]);
	bull.setZ(-pos[2]);
}

void ConvertPosToBull(const Vector &pos, btVector3 &bull) {
	bull.setX(HL2BULL(pos.x));
	bull.setY(HL2BULL(pos.z));
	bull.setZ(-HL2BULL(pos.y));
}

void ConvertPosToHL(const btVector3 &pos, Vector &hl) {
	hl.x = BULL2HL(pos.x());
	hl.y = -BULL2HL(pos.z());
	hl.z = BULL2HL(pos.y());
}

void ConvertAABBToBull(const Vector &hlMins, const Vector &hlMaxs, btVector3 &bullMins, btVector3 &bullMaxs) {
	Assert(hlMins.x <= hlMaxs.x);
	Assert(hlMins.y <= hlMaxs.y);
	Assert(hlMins.z <= hlMaxs.z);

	Vector halfExtents = (hlMaxs - hlMins) / 2;
	btVector3 bullHalfExtents;
	ConvertPosToBull(halfExtents, bullHalfExtents);

	Vector center = hlMins + halfExtents;
	btVector3 bullCenter;
	ConvertPosToBull(center, bullCenter);

	// Half Life AABBs use different corners.
	bullHalfExtents.setZ(-bullHalfExtents.z());

	bullMins = bullCenter - bullHalfExtents;
	bullMaxs = bullCenter + bullHalfExtents;
}

void ConvertAABBToHL(const btVector3 &bullMins, const btVector3 &bullMaxs, Vector &hlMins, Vector &hlMaxs) {
	Assert(bullMins.x() <= bullMaxs.x());
	Assert(bullMins.y() <= bullMaxs.y());
	Assert(bullMins.z() <= bullMaxs.z());

	btVector3 halfExtents = (bullMaxs - bullMins) / 2;
	Vector hlHalfExtents;
	ConvertPosToHL(halfExtents, hlHalfExtents);

	btVector3 center = bullMins + halfExtents;
	Vector hlCenter;
	ConvertPosToHL(center, hlCenter);

	// Half Life AABBs use different corners.
	hlHalfExtents.y = -hlHalfExtents.y;

	hlMins = hlCenter - hlHalfExtents;
	hlMaxs = hlCenter + hlHalfExtents;
}

void ConvertDirectionToBull(const Vector &dir, btVector3 &bull) {
	bull.setX(dir.x);
	bull.setY(dir.z);
	bull.setZ(-dir.y);
}

void ConvertDirectionToHL(const btVector3 &dir, Vector &hl) {
	hl.x = dir.x();
	hl.y = -dir.z();
	hl.z = dir.y();
}

void ConvertForceImpulseToBull(const Vector &hl, btVector3 &bull) {
	return ConvertPosToBull(hl, bull);
}

void ConvertForceImpulseToHL(const btVector3 &bull, Vector &hl) {
	return ConvertPosToHL(bull, hl);
}

void ConvertForceImpulseToBull(const float &hl, btScalar &bull) {
	bull = HL2BULL(hl);
}

void ConvertForceImpulseToHL(const btScalar &bull, float &hl) {
	hl = BULL2HL(bull);
}

void ConvertRotationToBull(const QAngle &angles, btMatrix3x3 &bull) {
	btQuaternion quat;
	ConvertRotationToBull(angles, quat);
	bull.setRotation(quat);
}

void ConvertRotationToBull(const QAngle &angles, btQuaternion &bull) {
	RadianEuler radian(angles);
	Quaternion q(radian);
	bull.setValue(q.x, q.z, -q.y, q.w);
}

void ConvertRotationToHL(const btMatrix3x3 &matrix, QAngle &hl) {
	btQuaternion quat;
	matrix.getRotation(quat);
	ConvertRotationToHL(quat, hl);
}

void ConvertRotationToHL(const btQuaternion &quat, QAngle &hl) {
	Quaternion q(quat.getX(), -quat.getZ(), quat.getY(), quat.getW());
	RadianEuler radian(q);
	hl = radian.ToQAngle();
}

void ConvertAngularImpulseToBull(const AngularImpulse &angularimp, btVector3 &bull) {
	bull.setX(DEG2RAD(angularimp.x));
	bull.setY(DEG2RAD(angularimp.z));
	bull.setZ(-DEG2RAD(angularimp.y));
}

void ConvertAngularImpulseToHL(const btVector3 &angularimp, AngularImpulse &hl) {
	hl.x = RAD2DEG(angularimp.x());
	hl.y = -RAD2DEG(angularimp.z());
	hl.z = RAD2DEG(angularimp.y());
}

void ConvertMatrixToHL(const btTransform &transform, matrix3x4_t &hl) {
	Vector forward, left, up, pos;

	ConvertDirectionToHL(transform.getBasis().getColumn(0), forward);
	ConvertDirectionToHL(-transform.getBasis().getColumn(2), left);
	ConvertDirectionToHL(transform.getBasis().getColumn(1), up);
	ConvertPosToHL(transform.getOrigin(), pos);

	hl.Init(forward, left, up, pos);
}

void ConvertMatrixToBull(const matrix3x4_t &hl, btTransform &transform) {
	Vector forward, left, up, pos;

	forward.x = hl[0][0];
	forward.y = hl[1][0];
	forward.z = hl[2][0];

	left.x = hl[0][1];
	left.y = hl[1][1];
	left.z = hl[2][1];

	up.x = hl[0][2];
	up.y = hl[1][2];
	up.z = hl[2][2];

	pos.x = hl[0][3];
	pos.y = hl[1][3];
	pos.z = hl[2][3];

	btVector3 bullForward, bullRight, bullUp, origin;
	ConvertDirectionToBull(forward, bullForward);
	ConvertDirectionToBull(-left, bullRight);
	ConvertDirectionToBull(up, bullUp);
	ConvertPosToBull(pos, origin);

	transform.setBasis(btMatrix3x3(bullForward.x(), bullUp.x(), bullRight.x(),
								   bullForward.y(), bullUp.y(), bullRight.y(),
								   bullForward.z(), bullUp.z(), bullRight.z()));
	transform.setOrigin(origin);
}

float ConvertDistanceToBull(float distance) {
	return HL2BULL(distance);
}

float ConvertDistanceToHL(float distance) {
	return BULL2HL(distance);
}

float ConvertEnergyToHL(float energy) {
	return energy * HL2BULL_INSQR_PER_METERSQR;
}
