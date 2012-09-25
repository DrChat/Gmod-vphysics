#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "LinearMath/btIDebugDraw.h"

struct SDL_Surface;

class GLDebugDrawer : public btIDebugDraw {
	int					m_debugMode;
	btCollisionWorld *	m_world;
	SDL_Surface *		Display;
public:
			GLDebugDrawer(btCollisionWorld* world);
			~GLDebugDrawer();

	void	drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor);
	void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color);

	void	drawSphere (const btVector3& p, btScalar radius, const btVector3& color);
	void	drawBox (const btVector3& boxMin, const btVector3& boxMax, const btVector3& color, btScalar alpha);

	void	drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha);
	
	void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);

	void	reportErrorWarning(const char* warningString);

	void	draw3dText(const btVector3& location,const char* textString);

	void	setDebugMode(int debugMode);

	int		getDebugMode() const { return m_debugMode;}

	void	DrawWorld();
};

#endif//GL_DEBUG_DRAWER_H
