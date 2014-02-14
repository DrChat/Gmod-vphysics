#ifndef PHYSICS_SOFTBODY_H
#define PHYSICS_SOFTBODY_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include "vphysics/softbodyV32.h"

// Purpose: Dynamically deforming meshes (blankets, dents in objects, etc.)

// Class declarations
class CPhysicsEnvironment;
class CPhysicsObject;

class btSoftBody;

class CPhysicsSoftBody : public IPhysicsSoftBody {
	public:
		CPhysicsSoftBody();
		~CPhysicsSoftBody();

		void			SetTotalMass(float fMass, bool bFromFaces = false);
		void			Anchor(int node, IPhysicsObject *pObj);

		int				GetNodeCount() const;
		int				GetFaceCount() const;
		int				GetLinkCount() const;

		softbodynode_t	GetNode(int i) const;
		softbodyface_t	GetFace(int i) const;
		softbodylink_t	GetLink(int i) const;

		void			SetNode(int i, softbodynode_t &node);

		// Get soft body AABB (cannot be implemented in collision interface because soft bodies change shape)
		void			GetAABB(Vector *mins, Vector *maxs) const;

		IPhysicsEnvironment32 *GetPhysicsEnvironment() const { return (IPhysicsEnvironment32 *)m_pEnv; }

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
CPhysicsSoftBody *CreateSoftBodyRope(CPhysicsEnvironment *pEnv, const Vector &position, const Vector &length, const softbodyparams_t *pParams);

#endif // PHYSICS_SOFTBODY_H