#include "StdAfx.h"

#include "GLDebugDrawer.h"

#include <SDL.h>
#include <SDL_opengl.h>

// memdbgon must be the last include file in a .cpp file!!!
//#include "tier0/memdbgon.h"

#if DEBUG_DRAW == 1
#pragma comment(lib, "SDL")
#pragma comment(lib, "OpenGL32")
#pragma comment(lib, "Glu32")

GLDebugDrawer::GLDebugDrawer(btCollisionWorld* world) : m_debugMode(0) {
	SDL_Init(SDL_INIT_VIDEO);
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	Display = SDL_SetVideoMode(640, 480, 32, SDL_OPENGL);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glClearColor(0, 0, 0, 0);
	glViewport(0, 0, 640, 480);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1, 1, -1, 1, 1, 10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 17, -47, 0, 0, 0, 0, 1, 0);

	m_world = world;
	m_world->setDebugDrawer(this);
}

GLDebugDrawer::~GLDebugDrawer() {
	m_world->setDebugDrawer(NULL);
	SDL_FreeSurface(Display);
	SDL_Quit();
}

void GLDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor) {
	glBegin(GL_LINES);
		glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
	glEnd();
}

void GLDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color) {
	drawLine(from,to,color,color);
}

void GLDebugDrawer::drawSphere (const btVector3& p, btScalar radius, const btVector3& color) {
	glColor4f (color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
	glPushMatrix ();
	glTranslatef (p.getX(), p.getY(), p.getZ());

	int lats = 5;
	int longs = 5;

	int i, j;
	for(i = 0; i <= lats; i++) {
		btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar) (i - 1) / lats);
		btScalar z0  = radius*sin(lat0);
		btScalar zr0 =  radius*cos(lat0);

		btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar) i / lats);
		btScalar z1 = radius*sin(lat1);
		btScalar zr1 = radius*cos(lat1);

		glBegin(GL_QUAD_STRIP);
		for(j = 0; j <= longs; j++) {
			btScalar lng = 2 * SIMD_PI * (btScalar) (j - 1) / longs;
			btScalar x = cos(lng);
			btScalar y = sin(lng);

			glNormal3f(x * zr0, y * zr0, z0);
			glVertex3f(x * zr0, y * zr0, z0);
			glNormal3f(x * zr1, y * zr1, z1);
			glVertex3f(x * zr1, y * zr1, z1);
		}
		glEnd();
	}

	glPopMatrix();
}

void GLDebugDrawer::drawBox (const btVector3& boxMin, const btVector3& boxMax, const btVector3& color, btScalar alpha) {
	btVector3 halfExtent = (boxMax - boxMin) * btScalar(0.5f);
	btVector3 center = (boxMax + boxMin) * btScalar(0.5f);
	//glEnable(GL_BLEND);     // Turn blending On
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glColor4f (color.getX(), color.getY(), color.getZ(), alpha);
	glPushMatrix ();
	glTranslatef (center.getX(), center.getY(), center.getZ());
	glScaled(2*halfExtent[0], 2*halfExtent[1], 2*halfExtent[2]);
	glPopMatrix ();
	//glDisable(GL_BLEND);
}

void GLDebugDrawer::drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha) {
	{
		const btVector3	n=btCross(b-a,c-a).normalized();
		glBegin(GL_TRIANGLES);		
		glColor4f(color.getX(), color.getY(), color.getZ(),alpha);
		glNormal3d(n.getX(),n.getY(),n.getZ());
		glVertex3d(a.getX(),a.getY(),a.getZ());
		glVertex3d(b.getX(),b.getY(),b.getZ());
		glVertex3d(c.getX(),c.getY(),c.getZ());
		glEnd();
	}
}

void GLDebugDrawer::setDebugMode(int debugMode) {
	m_debugMode = debugMode;
}

void GLDebugDrawer::draw3dText(const btVector3& location,const char* textString) {
	glRasterPos3f(location.x(),  location.y(),  location.z());
}

void GLDebugDrawer::reportErrorWarning(const char* warningString) {
	printf("%s\n",warningString);
}

void GLDebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) {
	{
		btVector3 to=pointOnB+normalOnB*1;//distance;
		const btVector3&from = pointOnB;
		glColor4f(color.getX(), color.getY(), color.getZ(),1.f);
		glBegin(GL_LINES);
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		glEnd();

	}
}

void GLDebugDrawer::DrawWorld() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	setDebugMode(DBG_DrawWireframe);
	m_world->debugDrawWorld();
	glFlush();
	SDL_GL_SwapBuffers();
}
#endif // DEBUG_DRAW