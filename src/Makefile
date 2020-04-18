BIN = ../bin

ifndef LIVE_INCLUDE
	LIVE_INCLUDE = /usr/local/include
endif
ifndef LIVE_LIB
	LIVE_LIB = /usr/local/lib
endif

INCLUDE = ../include
INCLUDES = -I$(INCLUDE) -I$(LIVE_INCLUDE)/UsageEnvironment -I$(LIVE_INCLUDE)/groupsock/ -I$(LIVE_INCLUDE)/liveMedia -I$(LIVE_INCLUDE)/BasicUsageEnvironment
# Default library filename suffixes for each library that we link with.  The "config.*" file might redefine these later.
libliveMedia_LIB_SUFFIX = $(LIB_SUFFIX)
libBasicUsageEnvironment_LIB_SUFFIX = $(LIB_SUFFIX)
libUsageEnvironment_LIB_SUFFIX = $(LIB_SUFFIX)
libgroupsock_LIB_SUFFIX = $(LIB_SUFFIX)
##### Change the following for your environment:
MARCOS = -DSOCKLEN_T=socklen_t -g -D_LARGEFILE_SOURCE=1 -D_FILE_OFFSET_BITS=64 -DDEBUG_RTP_TIMESTAMP=1 -DDEBUG_TIMESTAMPS=1\
	-DGL_RENDOR_SPHERE=1
COMPILE_OPTS =	$(INCLUDES) $(MARCOS)
COMPILE_OPTS += $$(pkg-config --cflags libavcodec libavutil libavformat libswscale libglog glfw3)
C =			c
C_COMPILER =		cc
C_FLAGS =		$(COMPILE_OPTS)
CPP =			cpp
CPLUSPLUS_COMPILER =	c++
CPLUSPLUS_FLAGS =	$(COMPILE_OPTS) -Wall -DBSD=1 -std=c++11
OBJ =			o
LINK =			c++ -o
LINK_OPTS =		-L/usr/local/lib
LIB_SUFFIX =			a
LIBS_FOR_CONSOLE_APPLICATION = -lssl -lcrypto -lavcodec -lavutil -lavformat -lswscale -lglog -ljsoncpp -lglfw3 -ldl -lGL -lpthread 
LIBS_FOR_GUI_APPLICATION =
EXE =
##### End of variables to change

DESTDIR = /usr/local

all: abr$(EXE)
	mkdir -p $(BIN)
	cp abr$(EXE) $(BIN)
	cp fragment_shader.glsl vertex_shader.glsl $(BIN)

.$(C).$(OBJ):
	$(C_COMPILER) -c $(C_FLAGS) $<
.$(CPP).$(OBJ):
	$(CPLUSPLUS_COMPILER) -c $(CPLUSPLUS_FLAGS) $<

GL_OBJS = gl.$(OBJ) glad.$(OBJ) Sphere.$(OBJ)
ABR_OBJS = abr.$(OBJ) TileAgg.$(OBJ) OurUsageEnvironment.$(OBJ) rtsp.$(OBJ) $(GL_OBJS)

abr.$(CPP): $(INCLUDE)/TileAgg.hh $(INCLUDE)/OurUsageEnvironment.hh $(INCLUDE)/config.hh $(INCLUDE)/ToString.hh $(INCLUDE)/rtsp.hh

USAGE_ENVIRONMENT_LIB = $(LIVE_LIB)/libUsageEnvironment.$(libUsageEnvironment_LIB_SUFFIX)
BASIC_USAGE_ENVIRONMENT_LIB = $(LIVE_LIB)/libBasicUsageEnvironment.$(libBasicUsageEnvironment_LIB_SUFFIX)
LIVEMEDIA_LIB = $(LIVE_LIB)/libliveMedia.$(libliveMedia_LIB_SUFFIX)
GROUPSOCK_LIB = $(LIVE_LIB)/libgroupsock.$(libgroupsock_LIB_SUFFIX)
LOCAL_LIBS =	$(LIVEMEDIA_LIB) $(GROUPSOCK_LIB) \
		$(BASIC_USAGE_ENVIRONMENT_LIB) $(USAGE_ENVIRONMENT_LIB)
LIBS =			$(LOCAL_LIBS) $(LIBS_FOR_CONSOLE_APPLICATION)

abr$(EXE): $(ABR_OBJS)  $(LOCAL_LIBS)
	$(LINK)$@ $(LINK_OPTS) $(ABR_OBJS) $(LIBS) 

clean:
	-rm -rf *.$(OBJ) *~ $(INCLUDE)/*~

install: $(ALL)
	  install -d $(DESTDIR)/bin
	  install -m 755 $(ALL) $(DESTDIR)/bin