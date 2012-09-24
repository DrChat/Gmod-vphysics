#include <ctype.h>
#include <tier1/tier1.h>
#include <tier1/KeyValues.h>
#include <tier1/utlsymbol.h>
#include <vphysics_interface.h>
#include <vphysics/collision_set.h>
#include <vphysics/constraints.h>
#include <vphysics/friction.h>
#include <vphysics/object_hash.h>
#include <vphysics/performance.h>
#include <vphysics/player_controller.h>
#include <vphysics/stats.h>
#include <vphysics/vehicles.h>
#include <vphysics/virtualmesh.h>
#include <vcollide_parse.h>
#include <cmodel.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btMaterial.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#define DEBUG_DRAW 0

#define COLLISION_MARGIN 0.004

#define SAFE_DIVIDE(a, b) ((b) != 0 ? (a)/(b) : 0)

//extern IPhysics* g_ValvePhysics;
//extern IPhysicsCollision* g_ValvePhysicsCollision;
#ifdef _DEBUG
//#define NOT_IMPLEMENTED __asm {int 3}

#define NOT_IMPLEMENTED Msg("VPhysics UNIMPLEMENTED: %s (%s:%d)\n", __FUNCTION__, __FILE__, __LINE__);
#else
#define NOT_IMPLEMENTED Msg("VPhysics UNIMPLEMENTED: %s (%s:%d)\n", __FUNCTION__, __FILE__, __LINE__);
#endif

// Putting these in here because I dont want to make a header just for them
struct btMassCenterMotionState : public btMotionState
{
	btTransform	m_centerOfMassOffset;
	btTransform m_worldTrans;
	void*		m_userPointer;

	btMassCenterMotionState(const btTransform& startTrans = btTransform::getIdentity(),const btTransform& centerOfMassOffset = btTransform::getIdentity())
		: m_centerOfMassOffset(centerOfMassOffset), m_worldTrans(startTrans * centerOfMassOffset), m_userPointer(0)
	{
	}

	virtual void getWorldTransform(btTransform& worldTrans) const { worldTrans = m_worldTrans; }
	virtual void setWorldTransform(const btTransform& worldTrans) { m_worldTrans = worldTrans; }
	virtual void getGraphicTransform(btTransform& graphTrans) const { graphTrans = m_worldTrans * m_centerOfMassOffset.inverse(); }
	virtual void setGraphicTransform(const btTransform& graphTrans) { m_worldTrans = graphTrans * m_centerOfMassOffset; }
};

struct PhysicsShapeInfo
{
	btVector3 massCenter;
};