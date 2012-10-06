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
	/*
	Vector forward, right, up;
	btVector3 bullForward, bullRight, bullUp;
	
	AngleVectors( angles, &forward, &right, &up );
	// now this is left
	//right = -right;

	up = -up;
	
	ConvertDirectionToBull( forward, bullForward );
	ConvertDirectionToBull( right, bullRight );
	ConvertDirectionToBull( up, bullUp );

	bull.setValue(
		bullForward.x(), bullForward.y(), bullForward.z(),
		bullUp.x(), bullUp.y(), bullUp.z(),
		bullRight.x(), bullRight.y(), bullRight.z()
		);
	*/
	
	RadianEuler radian(angles);
	Quaternion q(radian);
	btQuaternion quat(q.x, q.z, -q.y, q.w);
	bull.setRotation(quat);
}

void ConvertRotationToBull(const QAngle &angles, btQuaternion &bull) {
	/*
	btMatrix3x3 temp;
	ConvertRotationToBull(angles, temp);
	bull = btQuaternion(temp);
	*/
	
	RadianEuler radian(angles);
	Quaternion q(radian);
	bull.setValue(q.x, q.z, -q.y, q.w);
}

// NOTE: This function is BROKEN! Rewrite this!
// You can test this with:
//	lua_run local ent = player.GetByID(1):GetEyeTrace().Entity local i = 0 hook.Add("Think", "asdft", function() if i > 180 then i = -180 end print(tostring(ent:GetAngles()) .. " i(" .. i .. ")") ent:SetAngles(Angle(i, 0, 0)) i = i + 1 end)
// With this code, when HL.x > 90 || HL.x < -90 the other two angles (y and z) start screwing over.
void ConvertRotationToHL(const btMatrix3x3 &matrix, QAngle &hl) {
	btVector3 out;
	Vector forward, right, up;

	out = matrix.getColumn(0);
	ConvertDirectionToHL(out, forward);
	out = matrix.getColumn(2);
	ConvertDirectionToHL(out, right);
	out = matrix.getColumn(1);
	ConvertDirectionToHL(out, up);

	up = -up;

	float xyDist = sqrt( forward.x * forward.x + forward.y * forward.y );
	
	// enough here to get angles?
	if ( xyDist > 0.001 ) {
		// (yaw)	y = ATAN( forward.y, forward.x );		-- in our space, forward is the X axis
		hl.y = RAD2DEG( atan2( forward.y, forward.x ) );

		// (pitch)	x = ATAN( -forward.z, sqrt(forward.x*forward.x+forward.y*forward.y) );
		hl.x = RAD2DEG( atan2( -forward.z, xyDist ) );

		// (roll)	z = ATAN( -right.z, up.z );
		hl.z = RAD2DEG( atan2( -right.z, up.z ) ) + 180;
	} else {	// forward is mostly Z, gimbal lock
		// (yaw)	y = ATAN( -right.x, right.y );		-- forward is mostly z, so use right for yaw
		hl.y = RAD2DEG( atan2( right.x, -right.y ) );

		// (pitch)	x = ATAN( -forward.z, sqrt(forward.x*forward.x+forward.y*forward.y) );
		hl.x = RAD2DEG( atan2( -forward.z, xyDist ) );

		// Assume no roll in this case as one degree of freedom has been lost (i.e. yaw == roll)
		hl.z = 180;
	}
	
	/*
	btQuaternion quat;
	matrix.getRotation(quat);
	Quaternion q(quat.getX(), -quat.getZ(), quat.getY(), quat.getW());
	RadianEuler radian(q);
	hl = radian.ToQAngle();
	*/
}

void ConvertRotationToHL(const btQuaternion &quat, QAngle &hl) {
	btMatrix3x3 temp(quat);
	ConvertRotationToHL(temp, hl);

	/*
	Quaternion q(quat.getX(), -quat.getZ(), quat.getY(), quat.getW());
	RadianEuler radian(q);
	hl = radian.ToQAngle();
	*/
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
