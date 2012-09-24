#include "StdAfx.h"

#include "CPhysicsTrace.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

/************************************************
* CLASSES
************************************************/
// CTraceBull
class CTraceBull: public ITraceObject {
	public:
		CTraceBull(const btCollisionShape *pShape, const Vector &origin, const QAngle &angles, CPhysicsTrace &traceapi);

		virtual float Radius() {
			return m_radius;
		}

		btCollisionShape *		m_pShape;
	private:
		CPhysicsTrace &			m_traceapi;
		float					m_radius;
};

CTraceBull::CTraceBull(const btCollisionShape *pShape, const Vector &origin, const QAngle &angles, CPhysicsTrace &traceapi):
						m_traceapi(traceapi)
{
	
}

// CPhysicsTrace
CPhysicsTrace::CPhysicsTrace()
{
}

CPhysicsTrace::~CPhysicsTrace()
{
}

Vector CPhysicsTrace::GetExtent(const btCollisionShape *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction)
{
	NOT_IMPLEMENTED
	return collideOrigin;
}