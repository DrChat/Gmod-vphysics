#include "StdAfx.h"

#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
////#include "tier0/memdbgon.h"

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

void ConvertRotationToBull(const QAngle &angles, btMatrix3x3 &bull) {
	RadianEuler radian(angles);
	Quaternion q(radian);
	btQuaternion quat(q.x, q.z, -q.y, q.w);
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
	Quaternion q(quat.getX(), -quat.getZ(), quat.getY(), quat.getW());
	RadianEuler radian(q);
	hl = radian.ToQAngle();
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

	btVector3 bullForward, bullLeft, bullUp, origin;
	ConvertDirectionToBull(forward, bullForward);
	ConvertDirectionToBull(-left, bullLeft);
	ConvertDirectionToBull(up, bullUp);
	ConvertPosToBull(pos, origin);

	transform.setBasis(btMatrix3x3(bullForward.x(), bullUp.x(), bullLeft.x(), bullForward.y(), bullUp.y(), bullLeft.y(), bullForward.z(), bullUp.z(), bullLeft.z()));
	transform.setOrigin(origin);
}

float ConvertDistanceToBull(float distance) {
	return HL2BULL(distance);
}

float ConvertDistanceToHL(float distance) {
	return BULL2HL(distance);
}

float ConvertEnergyToHL( float energy )
{
	return energy * HL2BULL_INSQR_PER_METERSQR;
}
