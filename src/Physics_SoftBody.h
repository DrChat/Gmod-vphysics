#ifndef PHYSICS_SOFTBODY_H
#define PHYSICS_SOFTBODY_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

// Purpose: Dynamically deforming meshes (blankets, dents in objects, etc.)

// TODO: Expose later
struct softbodyparams_t {

};

// Class declarations
class CPhysicsEnvironment;

class btSoftBody;

class CPhysicsSoftBody : public IPhysicsSoftBody {
	public:
		CPhysicsSoftBody();
		~CPhysicsSoftBody();

		void			SetTotalMass(float fMass, bool bFromFaces);

		// UNEXPOSED FUNCTIONS
	public:
		void			Init(CPhysicsEnvironment *pEnv, btSoftBody *pSoftBody);

		btSoftBody *	GetSoftBody();

	private:
		CPhysicsEnvironment *	m_pEnv;
		btSoftBody *			m_pSoftBody;
};

CPhysicsSoftBody *CreateSoftBodyFromTriMesh(CPhysicsEnvironment *pEnv); // TODO: Not complete
CPhysicsSoftBody *CreateSoftBodyFromVertices(CPhysicsEnvironment *pEnv, const Vector *vertices, int numVertices, const Vector &position, const QAngle &angles, const softbodyparams_t *pParams);

#endif // PHYSICS_SOFTBODY_H