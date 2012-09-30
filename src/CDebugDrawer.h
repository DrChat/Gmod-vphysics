#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "LinearMath/btIDebugDraw.h"

#define RENDER_SDL 0

class CPhysicsEnvironment;
struct SDL_Surface;

class CDebugDrawer : public btIDebugDraw {
	int						m_debugMode;
	btCollisionWorld *		m_world;
	SDL_Surface *			Display;
	IVPhysicsDebugOverlay *	m_overlay;

	CPhysicsEnvironment *	m_pEnv;
public:
	CDebugDrawer(btCollisionWorld *world, CPhysicsEnvironment *pEnv);
	virtual ~CDebugDrawer();

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor);
	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color);

#if RENDER_SDL
	virtual void	drawSphere (const btVector3& p, btScalar radius, const btVector3& color);
#endif
	virtual void	drawBox (const btVector3& boxMin, const btVector3& boxMax, const btVector3& color, btScalar alpha);

	virtual void	drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha);
	
	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);

	virtual void	reportErrorWarning(const char* warningString);

	virtual void	draw3dText(const btVector3& location,const char* textString);

	virtual void	setDebugMode(int debugMode);

	virtual int		getDebugMode() const { return m_debugMode;}

	void DrawWorld();
};

#endif//GL_DEBUG_DRAWER_H
