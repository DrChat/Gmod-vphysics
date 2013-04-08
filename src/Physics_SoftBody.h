#ifndef CPHYSICSSOFTBODY_H
#define CPHYSICSSOFTBODY_H

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

		// Unexposed functions
	public:
		void	Init(CPhysicsEnvironment *pEnv, btSoftBody *pSoftBody);

	private:
		CPhysicsEnvironment *	m_pEnv;
		btSoftBody *			m_pSoftBody;
};

CPhysicsSoftBody *CreateSoftBodyFromTriMesh(CPhysicsEnvironment *pEnv);

#endif // CPHYSICSSOFTBODY_H