# My very first makefile!
MODULE_NAME = vphysics_srv.so
PROJECT_NAME = vphysics

# Configuration (can only be "debug" or "release")
CONFIGURATION = release

# Path Configuration
# Doesn't matter whether it's mp or sp.
SOURCE_SDK = ../thirdparty/sourcesdk/mp/src
BULLET_SDK = ../bullet

# Note: Srcds is required for a successful build because we need to link to some shared objects.
SRCDS_BIN_DIR = ~/steamcmd/server/bin

PROJECT_DIR = .
OUT_DIR = ../build/bin/linux/$(CONFIGURATION)
OBJ_DIR = ../build/obj/linux/$(PROJECT_NAME)/$(CONFIGURATION)

# Compilation Configuration
INCLUDES = \
	-I../include 			\
	-I$(SOURCE_SDK)/public 		\
	-I$(SOURCE_SDK)/public/tier0 	\
	-I$(SOURCE_SDK)/public/tier1 	\
	-I$(BULLET_SDK)/src
	
STATICLIBDIRS = \
	-L../build/lib/linux/$(CONFIGURATION)
	
# Only works in this order for whatever reason!
STATICLIBS = \
	-lBulletSoftBody 	\
	-lBulletDynamics 	\
	-lBulletMultiThreaded	\
	-lBulletCollision	\
	-lLinearMath
	
DYNAMICLIBS = \
	$(SOURCE_SDK)/lib/public/linux32/tier1.a 	\
	$(SOURCE_SDK)/lib/public/linux32/tier2.a 	\
	$(SOURCE_SDK)/lib/public/linux32/tier3.a 	\
	$(SOURCE_SDK)/lib/public/linux32/mathlib.a 	\
	$(SRCDS_BIN_DIR)/libvstdlib_srv.so 		\
	$(SRCDS_BIN_DIR)/libtier0_srv.so
		
CC = /usr/bin/g++
LINK = /usr/bin/g++
ARCH = i386
DEFINES = -DLINUX -D__LINUX__ -D_LINUX -D__linux__ -DPOSIX -DGNUC -DARCH=$(ARCH) -Dsprintf_s=snprintf -Dstrcmpi=strcasecmp -D_alloca=alloca -Dstricmp=strcasecmp -D_stricmp=strcasecmp -Dstrcpy_s=strncpy -D_strnicmp=strncasecmp -Dstrnicmp=strncasecmp -D_snprintf=snprintf -D_vsnprintf=vsnprintf -D_alloca=alloca -Dstrcmpi=strcasecmp -DNO_MALLOC_OVERRIDE
CFLAGS = $(INCLUDES) $(DEFINES) -fpermissive -fPIC -w -msse2 -m32 -march=$(ARCH) -g
LFLAGS = -m32 -msse2 -lm -ldl $(STATICLIBDIRS) -Wl,-Bstatic $(STATICLIBS) -Wl,-Bdynamic $(DYNAMICLIBS) -shared

ifeq ($(CONFIGURATION), debug)
	# Optimize but don't affect debugging experience. (-Og, only on G++ 4.8 and above)
	#CFLAGS += -Og
	CFLAGS += -D_DEBUG -O0
endif

ifeq ($(CONFIGURATION), release)
	CFLAGS += -O2 -DNDEBUG
endif

SOURCES = $(wildcard *.cpp)
OBJECTS = $(addprefix $(OBJ_DIR)/, $(SOURCES:.cpp=.o))

# Commands
RM = rm -f
CP = cp -f
MKDIR = mkdir -p
MAKE = make

all: dirs $(MODULE_NAME) copy
build: dirs $(MODULE_NAME)

dirs:
	@-$(MKDIR) $(OUT_DIR)
	@-$(MKDIR) $(OBJ_DIR)
	
copy: $(MODULE_NAME)
	@echo " + Copying $(OUT_DIR)/$(MODULE_NAME) to $(SRCDS_BIN_DIR)"
	@$(CP) $(OUT_DIR)/$(MODULE_NAME) $(SRCDS_BIN_DIR)
		
$(MODULE_NAME): $(OBJECTS)
	@echo " + Linking..."
	@$(LINK) -o $(OUT_DIR)/$@ $(OBJECTS) $(LFLAGS)
	@echo " + Module created at $(OUT_DIR)/$@"

$(OBJ_DIR)/%.o: %.cpp
	@echo " + Compiling '$<'"
	@$(CC) $(CFLAGS) -o $@ -c $<
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(OUT_DIR)/$(MODULE_NAME)
	@echo " + Clean!"
  
