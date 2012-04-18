#include <ctype.h>
#include <tier1/tier1.h>
#include <tier1/KeyValues.h>
#include <tier1/utlsymbol.h>
#include <vphysics_interface.h>
#include <vphysics/collision_set.h>
#include <vphysics/friction.h>
#include <vphysics/object_hash.h>
#include <vphysics/performance.h>
#include <vphysics/player_controller.h>
#include <vphysics/virtualmesh.h>
#include <cmodel.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btMaterial.h>

extern IPhysics* g_ValvePhysics;
extern IPhysicsCollision* g_ValvePhysicsCollision;
#ifdef _DEBUG
#define NOT_IMPLEMENTED __asm {int 3}
#else
#define NOT_IMPLEMENTED
#endif
