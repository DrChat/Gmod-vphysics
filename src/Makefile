# My very first makefile!
MODULE_NAME = vphysics_srv.so
PROJECT_NAME = vphysics

# Path Configuration
SOURCE_SDK = ../../../../Libraries/SourceMod/src
BULLET_SDK = ../bullet
# Note: Srcds is required for a successful build because we need to link to some shared objects.
SRCDS_BIN_DIR = ~/srcds/orangebox/bin

PROJECT_DIR = .
OUT_DIR = ../build/bin/linux/Release
OBJ_DIR = ../build/obj/linux/$(PROJECT_NAME)/Release

BULLET_MAKEFILE_DIR = ../bullet/make

# Compilation Configuration
INCLUDES = \
	-I../include \
	-I$(SOURCE_SDK)/public \
	-I$(SOURCE_SDK)/public/tier0 \
	-I$(SOURCE_SDK)/public/tier1 \
	-I$(BULLET_SDK)/src
	
STATICLIBDIRS = \
	-L../build/lib/linux/Release
	
STATICLIBS = \
	-lBulletSoftBody \
	-lBulletDynamics \
	-lBulletCollision \
	-lLinearMath
	
DYNAMICLIBS = \
	$(SOURCE_SDK)/lib/linux/tier1_i486.a \
	$(SOURCE_SDK)/lib/linux/mathlib_i486.a \
	$(SRCDS_BIN_DIR)/libvstdlib_srv.so \
	$(SRCDS_BIN_DIR)/libtier0_srv.so
		
CC = /usr/bin/g++-3.4
LINK = /usr/bin/g++-3.4
ARCH = i386
DEFINES = -DLINUX -D__LINUX__ -D_LINUX -D__linux__ -DARCH=$(ARCH) -Dsprintf_s=snprintf -Dstrcmpi=strcasecmp -D_alloca=alloca -Dstricmp=strcasecmp -D_stricmp=strcasecmp -Dstrcpy_s=strncpy -D_strnicmp=strncasecmp -Dstrnicmp=strncasecmp -D_snprintf=snprintf -D_vsnprintf=vsnprintf -D_alloca=alloca -Dstrcmpi=strcasecmp -DNDEBUG -DNO_MALLOC_OVERRIDE
CFLAGS = $(INCLUDES) $(DEFINES) -fpermissive -fPIC -w -msse2 -m32 -march=$(ARCH)
LFLAGS = -m32 -lm -ldl $(STATICLIBDIRS) -Wl,-Bstatic $(STATICLIBS) -Wl,-Bdynamic $(DYNAMICLIBS) -shared

SOURCES = $(wildcard *.cpp)
OBJECTS = $(addprefix $(OBJ_DIR)/, $(SOURCES:.cpp=.o))

# Commands
RM = rm -f
CP = cp -f
MKDIR = mkdir -p
MAKE = make

all: dirs bullet $(MODULE_NAME) copy
build: dirs bullet $(MODULE_NAME)

dirs:
	@-$(MKDIR) $(OUT_DIR)
	@-$(MKDIR) $(OBJ_DIR)
	
copy:
	@echo " + Copying $(OUT_DIR)/$(MODULE_NAME) to $(SRCDS_BIN_DIR)"
	@$(CP) $(OUT_DIR)/$(MODULE_NAME) $(SRCDS_BIN_DIR)
	
bullet:
	@echo " + Building bullet..."
	@$(MAKE) -f Makefile -C $(BULLET_MAKEFILE_DIR)
	
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
  