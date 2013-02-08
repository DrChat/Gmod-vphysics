# My very first makefile!
MODULE_NAME = vphysics.so

# Path Configuration
SOURCE_SDK = ../../../../Libraries/SourceMod/src
BULLET_SDK = ../../../../Libraries/bullet
# Unused as of now
SRCDS_DIR = .

LIBRARY_PATH = /usr/lib/x86_64-linux-gnu

PROJECT_DIR = .
OUT_DIR = ../build/bin/linux/Release
OBJ_DIR = ../build/obj/linux/vphysics/Release

# Compilation Configuration
INCLUDES = \
	-I../include \
	-I$(SOURCE_SDK)/public \
	-I$(SOURCE_SDK)/public/tier0 \
	-I$(SOURCE_SDK)/public/tier1 \
	-I$(BULLET_SDK)/src
	
LIBFILES = \
	$(SOURCE_SDK)/lib/linux/tier1_486.a \
	$(SOURCE_SDK)/lib/linux/tier2_486.a \
	$(BULLET_SDK)/buildlin-cmake/src/LinearMath/libLinearMath.a \
	$(BULLET_SDK)/buildlin-cmake/src/BulletCollision/libBulletCollision.a \
	$(BULLET_SDK)/buildlin-cmake/src/BulletDynamics/libBulletDynamics.a \
	$(BULLET_SDK)/buildlin-cmake/src/BulletSoftBody/libBulletSoftBody.a
		
CC = /usr/bin/g++-3.4
LINK = /usr/bin/g++-3.4
ARCH = i386
DEFINES = -DLINUX -D__LINUX__ -D_LINUX -D__linux__ -DARCH=$(ARCH) -Dsprintf_s=snprintf -Dstrcmpi=strcasecmp -D_alloca=alloca -Dstricmp=strcasecmp -D_stricmp=strcasecmp -Dstrcpy_s=strncpy -D_strnicmp=strncasecmp -Dstrnicmp=strncasecmp -D_snprintf=snprintf -D_vsnprintf=vsnprintf -D_alloca=alloca -Dstrcmpi=strcasecmp -DNDEBUG
CFLAGS = $(INCLUDES) $(DEFINES) -fpermissive -w -fPIC -m32
LFLAGS = $(CFLAGS) -lm -ldl -shared -Wl

SOURCES = $(wildcard *.cpp)
OBJECTS = $(addprefix $(OBJ_DIR)/, $(SOURCES:.cpp=.o))

# Commands
RM = rm -f

all: dirs $(MODULE_NAME)

dirs:
	@-mkdir -p $(OUT_DIR)
	@-mkdir -p $(OBJ_DIR)
	
$(MODULE_NAME): $(OBJECTS)
	@echo " + Linking..."
	$(CC) $(LFLAGS) -o $@ $(OBJECTS) $(LIBFILES)

$(OBJ_DIR)/%.o: %.cpp
	@echo " + Compiling '$<'"
	@$(CC) $(CFLAGS) -o $@ -c $<
	
clean:
	@$(RM) $(OBJECTS)
  