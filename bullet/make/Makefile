BULLET_SRC_DIR = ../src

BULLET_PROJS = BulletCollision BulletDynamics BulletMultiThreaded BulletSoftBody LinearMath
BULLET_OUT_DIR = ../../build/lib/linux/Release

# ------------------------ System Commands ------------------------
CC = /usr/bin/g++
LINK = /usr/bin/g++

# ------------------------ Compiler Settings ------------------------
INCLUDES = 
DEFINES = -DLINUX -D_LINUX -D__linux__ -d__LINUX__ -DNDEBUG

LIBS = -lm

CFLAGS = $(INCLUDES) $(DEFINES) -msse2 -m32 -march=i386
LFLAGS = $(LIBS) -msse2 -m32 -march=i386

# ------------------------ MAKE COMMANDS ------------------------
all: $(BULLET_PROJS)

# ------------------------ BulletCollision ------------------------
BULLETCOLLISION_DIR = $(BULLET_SRC_DIR)/BulletCollision

BULLETCOLLISION_SRCS = 	$(wildcard $(BULLETCOLLISION_DIR)/BroadphaseCollision/*.cpp)
BULLETCOLLISION_SRCS += $(wildcard $(BULLETCOLLISION_DIR)/CollisionDispatch/*.cpp)
BULLETCOLLISION_SRCS += $(wildcard $(BULLETCOLLISION_DIR)/CollisionShapes/*.cpp)
BULLETCOLLISION_SRCS += $(wildcard $(BULLETCOLLISION_DIR)/Gimpact/*.cpp)
BULLETCOLLISION_SRCS += $(wildcard $(BULLETCOLLISION_DIR)/NarrowPhaseCollision/*.cpp)

BULLETCOLLISION_OBJS = $(addprefix $(BULLET_OUT_DIR)/BulletCollision, $(BULLETCOLLISION_SRCS:.cpp=.o))

BulletCollision:
	@for obj in $(BULLETCOLLISION_OBJS); do	\
		echo $$obj;							\
	done

# ------------------------ BulletDynamics ------------------------
BULLETDYNAMICS_DIR = $(BULLET_SRC_DIR)/BulletDynamics

BULLETDYNAMICS_SRCS =	$(wildcard $(BULLETDYNAMICS_DIR)/Character/*.cpp)
BULLETDYNAMICS_SRCS +=	$(wildcard $(BULLETDYNAMICS_DIR)/ConstraintSolver/*.cpp)
BULLETDYNAMICS_SRCS +=	$(wildcard $(BULLETDYNAMICS_DIR)/Dynamics/*.cpp)
BULLETDYNAMICS_SRCS +=	$(wildcard $(BULLETDYNAMICS_DIR)/Vehicle/*.cpp)

BulletDynamics:
	@echo "BulletDynamics"

# ------------------------ BulletMultiThreaded ------------------------
BULLETMULTITHREADED_DIR = $(BULLET_SRC_DIR)/BulletMultiThreaded

BULLETMULTITHREADED_SRCS =	$(wildcard $(BULLETMULTITHREADED_DIR)/*.cpp)
BULLETMULTITHREADED_SRCS +=	$(wildcard $(BULLETMULTITHREADED_DIR)/SpuNarrowPhaseCollisionTask/*.cpp)

BulletMultiThreaded:
	@echo "BulletMultiThreaded"

# ------------------------ BulletSoftBody ------------------------
BULLETSOFTBODY_DIR = $(BULLET_SRC_DIR)/BulletSoftBody

BULLETSOFTBODY_SRCS =	$(wildcard $(BULLETSOFTBODY_DIR)/*.cpp)

BulletSoftBody:
	@echo "BulletSoftBody"

# ------------------------ LinearMath ------------------------
LINEARMATH_DIR = $(BULLET_SRC_DIR)/LinearMath

LINEARMATH_SRCS =	$(wildcard $(LINEARMATH_DIR)/*.cpp)

LinearMath:
	@echo "LinearMath"