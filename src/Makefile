# My very first makefile!
MODULE_NAME = vphysics_srv.so
PROJECT_NAME = vphysics

# Configuration
CONFIGURATION = debug

# Path Configuration
SOURCE_SDK = ../../../../Libraries/SourceMod/src
BULLET_SDK = ../bullet

# Note: Srcds is required for a successful build because we need to link to some shared objects.
SRCDS_BIN_DIR = ~/steamcmd/server/bin

PROJECT_DIR = .
OUT_DIR = ../build/bin/linux/release
OBJ_DIR = ../build/obj/linux/$(PROJECT_NAME)/release

# Compilation Configuration
INCLUDES = \
	-I../include 			\
	-I$(SOURCE_SDK)/public 		\
	-I$(SOURCE_SDK)/public/tier0 	\
	-I$(SOURCE_SDK)/public/tier1 	\
	-I$(BULLET_SDK)/src
	
STATICLIBDIRS = \
	-L../build/lib/linux/release
	
STATICLIBS = \
	-lBulletSoftBody 	\
	-lBulletDynamics 	\
	-lBulletMultiThreaded	\
	-lBulletCollision	\
	-lLinearMath
	
DYNAMICLIBS = \
	$(SOURCE_SDK)/lib/linux/libtier1_i486.a 	\
	$(SOURCE_SDK)/lib/linux/libmathlib_i486.a 	\
	$(SRCDS_BIN_DIR)/libvstdlib_srv.so 		\
	$(SRCDS_BIN_DIR)/libtier0_srv.so
		
CC = /usr/bin/g++
LINK = /usr/bin/g++
ARCH = i386
DEFINES = -DLINUX -D__LINUX__ -D_LINUX -D__linux__ -DARCH=$(ARCH) -Dsprintf_s=snprintf -Dstrcmpi=strcasecmp -D_alloca=alloca -Dstricmp=strcasecmp -D_stricmp=strcasecmp -Dstrcpy_s=strncpy -D_strnicmp=strncasecmp -Dstrnicmp=strncasecmp -D_snprintf=snprintf -D_vsnprintf=vsnprintf -D_alloca=alloca -Dstrcmpi=strcasecmp -DNDEBUG -DNO_MALLOC_OVERRIDE
CFLAGS = $(INCLUDES) $(DEFINES) -fpermissive -fPIC -w -msse2 -m32 -march=$(ARCH) -g3
LFLAGS = -m32 -lm -ldl $(STATICLIBDIRS) -Wl,-Bstatic $(STATICLIBS) -Wl,-Bdynamic $(DYNAMICLIBS) -shared

ifeq ($(CONFIGURATION), debug)
	# Optimize but don't affect debugging experience. (-Og, only on G++ 4.8 and above)
	#CFLAGS += -Og
endif

ifeq ($(CONFIGURATION), release)
	CFLAGS += -O2
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
  
