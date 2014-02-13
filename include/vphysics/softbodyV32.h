#ifndef SOFTBODYV32_H
#define SOFTBODYV32_H
#ifdef _MSC_VER
#pragma once
#endif

struct softbodynode_t {
	Vector pos, vel;
	float invMass;
};

struct softbodyface_t {
	softbodynode_t nodes[3];
	Vector normal;
};

struct softbodyparams_t {
	float	totalMass;
};

class IPhysicsSoftBody {
	public:
		virtual ~IPhysicsSoftBody() {}

		virtual void	GetPosition(Vector *pos, QAngle *ang) const = 0;
		virtual void	SetPosition(const Vector *pos, const QAngle *ang) = 0;

		virtual void	SetTotalMass(float fMass, bool bFromFaces = false) = 0;
		virtual void	Anchor(int node, IPhysicsObject *pObj) = 0;

		virtual int		GetNodeCount() const = 0;
		virtual int		GetFaceCount() const = 0;

		virtual softbodynode_t	GetNode(int i) const = 0;
		virtual softbodyface_t	GetFace(int i) const = 0;

		virtual IPhysicsEnvironment32 *GetPhysicsEnvironment() const = 0;
};

#endif // SOFTBODYV32_H