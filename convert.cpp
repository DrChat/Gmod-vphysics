#include "StdAfx.h"

#include "convert.h"

btCompoundShape* ConvertMeshToBull(CPhysCollide* ivp) {
	Vector hlvec[3];
	btVector3 btvec;
	ICollisionQuery* query = g_ValvePhysicsCollision->CreateQueryModel(ivp);
	int convexcount = query->ConvexCount();
	btCompoundShape* bull = new btCompoundShape;
	for (int convex = 0; convex < convexcount; convex++) {
		int triangles = query->TriangleCount(convex);
		btConvexHullShape* btconvex = new btConvexHullShape;
		for (int i = 0; i < triangles; i++) {
			query->GetTriangleVerts(convex, i, hlvec);
			ConvertPosToBull(hlvec[0], btvec);
			btconvex->addPoint(btvec);
			ConvertPosToBull(hlvec[1], btvec);
			btconvex->addPoint(btvec);
			ConvertPosToBull(hlvec[2], btvec);
			btconvex->addPoint(btvec);
		}
		bull->addChildShape(btTransform::getIdentity(), btconvex);
	}
	g_ValvePhysicsCollision->DestroyQueryModel(query);
	return bull;
}

btVector3& inline_ConvertPosToBull(const Vector& pos)
{
	btVector3 bull;
	ConvertPosToBull(pos, bull);
	return bull;
}

void ConvertPosToBull(const Vector& pos, btVector3& bull) {
	bull.setX(HL2BULL(pos.x));
	bull.setY(HL2BULL(pos.z));
	bull.setZ(-HL2BULL(pos.y));
}

Vector& inline_ConvertPosToHL(const btVector3& pos)
{
	Vector hl;
	ConvertPosToHL(pos, hl);
	return hl;
}

void ConvertPosToHL(const btVector3& pos, Vector& hl) {
	hl.x = BULL2HL(pos.x());
	hl.y = -BULL2HL(pos.z());
	hl.z = BULL2HL(pos.y());
}

void ConvertDirectionToBull(const Vector& dir, btVector3& bull) {
	bull.setX(dir.x);
	bull.setY(dir.z);
	bull.setZ(-dir.y);
}

void ConvertDirectionToHL(const btVector3& dir, Vector& hl) {
	hl.x = -dir.x();
	hl.y = dir.z();
	hl.z = dir.y();
}

void ConvertRotationToBull(const QAngle& angles, btMatrix3x3& bull) {
	RadianEuler radian(angles);
	btQuaternion q1;
	q1.setEulerZYX(radian.z, radian.y, radian.x);
	btQuaternion q2(q1.getX(), q1.getZ(), -q1.getY(), q1.getW());
	bull.setRotation(q2);
}

void ConvertRotationToBull(const QAngle& angles, btQuaternion& bull) {
	RadianEuler radian(angles);
	btQuaternion q;
	q.setEulerZYX(radian.z, radian.y, radian.x);
	bull.setX(q.getX());
	bull.setY(q.getZ());
	bull.setZ(-q.getY());
	bull.setW(q.getW());
}

void ConvertRotationToHL(const btMatrix3x3& matrix, QAngle& hl) {
	btQuaternion quat;
	matrix.getRotation(quat);
	Quaternion q(quat.getX(), -quat.getZ(), quat.getY(), quat.getW());
	RadianEuler radian(q);
	hl = radian.ToQAngle();
}

void ConvertRotationToHL(const btQuaternion& quat, QAngle& hl)
{
	Quaternion q(quat.getX(), -quat.getZ(), quat.getY(), quat.getW());
	RadianEuler radian(q);
	hl = radian.ToQAngle();
}

void ConvertAngularImpulseToBull(const AngularImpulse& angularimp, btVector3& bull) {
	bull.setX(DEG2RAD(angularimp.x));
	bull.setY(DEG2RAD(angularimp.z));
	bull.setZ(-DEG2RAD(angularimp.y));
}

void ConvertAngularImpulseToHL(const btVector3& angularimp, AngularImpulse& hl) {
	hl.x = RAD2DEG(angularimp.x());
	hl.y = -RAD2DEG(angularimp.z());
	hl.z = RAD2DEG(angularimp.y());
}

void ConvertMatrixToHL(const btTransform& transform, matrix3x4_t& hl) {
	Vector forward, left, up, pos;

	ConvertDirectionToHL(transform.getBasis().getColumn(0), forward);
	ConvertDirectionToHL(transform.getBasis().getColumn(2), left);
	ConvertDirectionToHL(transform.getBasis().getColumn(1), up);
	ConvertPosToHL(transform.getOrigin(), pos);

	hl[0][0] = forward.x;
	hl[1][0] = forward.y;
	hl[2][0] = forward.z;

	hl[0][1] = left.x;
	hl[1][1] = left.y;
	hl[2][1] = left.z;

	hl[0][2] = up.x;
	hl[1][2] = up.y;
	hl[2][2] = up.z;

	hl[0][3] = pos.x;
	hl[1][3] = pos.y;
	hl[2][3] = pos.z;
}

float ConvertDistanceToHL(float distance) {
	return BULL2HL(distance);
}

float ConvertEnergyToHL( float energy )
{
	return energy * HL2BULL_INSQR_PER_METERSQR;
}
