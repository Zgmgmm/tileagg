LIVE_INCLUDE = /usr/local/include
LIVE_LIB = /usr/local/lib
INCLUDES = -Iinclude -I$(LIVE_INCLUDE)/UsageEnvironment -I$(LIVE_INCLUDE)/groupsock/ -I$(LIVE_INCLUDE)/liveMedia -I$(LIVE_INCLUDE)/BasicUsageEnvironment\
# Default library filename suffixes for each library that we link with.  The "config.*" file might redefine these later.
libliveMedia_LIB_SUFFIX = $(LIB_SUFFIX)
libBasicUsageEnvironment_LIB_SUFFIX = $(LIB_SUFFIX)
libUsageEnvironment_LIB_SUFFIX = $(LIB_SUFFIX)
libgroupsock_LIB_SUFFIX = $(LIB_SUFFIX)
##### Change the following for your environment:
COMPILE_OPTS =	$(INCLUDES)  -I/usr/local/include -I.  -DSOCKLEN_T=socklen_t -g -D_LARGEFILE_SOURCE=1 -D_FILE_OFFSET_BITS=64 -DDEBUG_RTP_TIMESTAMP=1 -DDEBUG_TIMESTAMPS=1
COMPILE_OPTS += $$(pkg-config --cflags sdl2 libavcodec libavutil libavformat libswscale libglog glfw3)	
C =			c
C_COMPILER =		cc
C_FLAGS =		$(COMPILE_OPTS)
CPP =			cpp
CPLUSPLUS_COMPILER =	c++
CPLUSPLUS_FLAGS =	$(COMPILE_OPTS) -Wall -DBSD=1 -std=c++11
OBJ =			o
LINK =			c++ -o
LINK_OPTS =		-L.
CONSOLE_LINK_OPTS =	$(LINK_OPTS) -L/usr/local/lib
LIBRARY_LINK =		ar cr 
LIBRARY_LINK_OPTS =	
LIB_SUFFIX =			a
LIBS_FOR_CONSOLE_APPLICATION = -lssl -lcrypto -lSDL2 -lavcodec -lavutil -lavformat -lswscale -lglog -ljsoncpp -lglfw3 -ldl -lGL -lpthread 
LIBS_FOR_GUI_APPLICATION =
EXE =
##### End of variables to change

PREFIX = /usr/local

all: tileagg$(EXE) abr$(EXE) #gl$(EXE)

.$(C).$(OBJ):
	$(C_COMPILER) -c $(C_FLAGS) $<
.$(CPP).$(OBJ):
	$(CPLUSPLUS_COMPILER) -c $(CPLUSPLUS_FLAGS) $<

TILE_AGG_OBJS = main.$(OBJ) TileAgg.$(OBJ) OurUsageEnvironment.$(OBJ)
ABR_OBJS = abr.$(OBJ) TileAgg.$(OBJ) OurUsageEnvironment.$(OBJ) rtsp.$(OBJ) $(GL_OBJS)
GL_OBJS = gl.$(OBJ) glad.$(OBJ) Sphere.$(OBJ)

TileAgg.$(CPP): include/TileAgg.hh
main.$(CPP): include/TileAgg.hh include/OurUsageEnvironment.hh
abr.$(CPP): include/TileAgg.hh include/OurUsageEnvironment.hh include/config.hh include/ToString.hh include/rtsp.hh

USAGE_ENVIRONMENT_LIB = $(LIVE_LIB)/libUsageEnvironment.$(libUsageEnvironment_LIB_SUFFIX)
BASIC_USAGE_ENVIRONMENT_LIB = $(LIVE_LIB)/libBasicUsageEnvironment.$(libBasicUsageEnvironment_LIB_SUFFIX)
LIVEMEDIA_LIB = $(LIVE_LIB)/libliveMedia.$(libliveMedia_LIB_SUFFIX)
GROUPSOCK_LIB = $(LIVE_LIB)/libgroupsock.$(libgroupsock_LIB_SUFFIX)
LOCAL_LIBS =	$(LIVEMEDIA_LIB) $(GROUPSOCK_LIB) \
		$(BASIC_USAGE_ENVIRONMENT_LIB) $(USAGE_ENVIRONMENT_LIB)
LIBS =			$(LOCAL_LIBS) $(LIBS_FOR_CONSOLE_APPLICATION)

tileagg$(EXE): $(TILE_AGG_OBJS)  $(LOCAL_LIBS)
	$(LINK)$@ $(CONSOLE_LINK_OPTS) $(TILE_AGG_OBJS) $(LIBS) 

abr$(EXE): $(ABR_OBJS)  $(LOCAL_LIBS)
	$(LINK)$@ $(CONSOLE_LINK_OPTS) $(ABR_OBJS) $(LIBS) 

clean:
	-rm -rf *.$(OBJ) $(ALL) core *.core *~ include/*~

install: $(ALL)
	  install -d $(DESTDIR)$(PREFIX)/bin
	  install -m 755 $(ALL) $(DESTDIR)$(PREFIX)/bin