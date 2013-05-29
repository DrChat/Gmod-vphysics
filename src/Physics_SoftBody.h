#ifndef PHYSICS_SOFTBODY_H
#define PHYSICS_SOFTBODY_H

// Purpose: Dynamically deforming meshes (blankets, dents in objects, etc.)

// Public interface to expose later
class IPhysicsSoftBody {
	public:
		virtual ~IPhysicsSoftBody() {}
};

// Class declarations
class CPhysicsEnvironment;

class btSoftBody;

class CPhysicsSoftBody : public IPhysicsSoftBody {
	public:
		CPhysicsSoftBody();
		~CPhysicsSoftBody();

		// UNEXPOSED FUNCTIONS
	public:
		void			Init(CPhysicsEnvironment *pEnv, btSoftBody *pSoftBody);

		btSoftBody *	GetSoftBody();

	private:
		CPhysicsEnvironment *	m_pEnv;
		btSoftBody *			m_pSoftBody;
};

CPhysicsSoftBody *CreateSoftBodyFromTriMesh(CPhysicsEnvironment *pEnv);
CPhysicsSoftBody *CreateSoftBodyFromVertices(CPhysicsEnvironment *pEnv, const Vector *vertices, int numVertices, const Vector &position, const QAngle &angles);

#endif // PHYSICS_SOFTBODY_H