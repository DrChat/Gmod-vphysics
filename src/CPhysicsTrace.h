#ifndef CPHYSICSTRACE_H
#define CPHYSICSTRACE_H

class Vector;

class ITraceObject {
	public:
		virtual Vector	SupportMap( const Vector &dir, int &index ) = 0;
		virtual Vector	GetVertByIndex( int index ) = 0;
		virtual float	Radius( void ) = 0;
};

class CPhysicsTrace {
	public:
		CPhysicsTrace();
		~CPhysicsTrace();

		Vector GetExtent(const btCollisionShape *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction);
};

#endif // CPHYSICSTRACE_H