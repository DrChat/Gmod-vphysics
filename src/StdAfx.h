#include <ctype.h>
#include <tier1/tier1.h>
#include <tier1/utlsymbol.h>

#include <vphysics_interface.h>
#include "vphysics_interfaceV32.h"
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

#if defined(_WIN32)
#	define DEBUG_DRAW 1
#endif

#define COLLISION_MARGIN 0.004 // 4 mm

#define SLEEP_LINEAR_THRESHOLD 0.1 // m/s
#define SLEEP_ANGULAR_THRESHOLD 0.4 // rad/s

#define SAFE_DIVIDE(a, b) ((b) != 0 ? (a)/(b) : 0)

#define NOT_IMPLEMENTED DevWarning("VPhysics UNIMPLEMENTED: %s (%s:%d)\n", __FUNCTION__, __FILE__, __LINE__);
#define NOT_IMPLEMENTED_CRITICAL Error("VPhysics UNIMPLEMENTED: %s (%s:%d)\n", __FUNCTION__, __FILE__, __LINE__);