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

void ConvertPosToBull(const Vector& pos, btVector3& bull) {
	bull.setX(-HL2BULL(pos.x));
	bull.setY(HL2BULL(pos.z));
	bull.setZ(HL2BULL(pos.y));
}

void ConvertPosToHL(const btVector3& pos, Vector& hl) {
	hl.x = -BULL2HL(pos.x());
	hl.y = BULL2HL(pos.z());
	hl.z = BULL2HL(pos.y());
}

void ConvertRotationToBull(const QAngle& angles, btMatrix3x3& bull) {
	RadianEuler radian(angles);
	bull.setEulerZYX(radian.y, radian.x, radian.z);
}

void ConvertRotationToBull(const QAngle& angles, btQuaternion& bull) {
	RadianEuler radian(angles);
	bull.setEulerZYX(radian.y, radian.x, radian.z);
}

void ConvertRotationToHL(const btMatrix3x3& matrix, QAngle& hl) {
	RadianEuler radian;
	matrix.getEulerZYX(radian.y, radian.x, radian.z);
	hl = radian.ToQAngle();
}

void ConvertRotationToHL(const btQuaternion& quat, QAngle& hl) {
	btMatrix3x3 temp;
	temp.setRotation(quat);
	ConvertRotationToHL(temp, hl);
}
float ConvertEnergyToHL( float energy )
{
	return energy * HL2BULL_INSQR_PER_METERSQR;
}