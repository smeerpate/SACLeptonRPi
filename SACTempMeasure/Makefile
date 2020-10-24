LEPTON_INC = -I/opt/lepton/leptonSDKEmb32PUB/

INCLUDES = $(LEPTON_INC)

CFLAGS = $(INCLUDES)
CPPFLAGS = -march=armv7-a -mtune=cortex-a7

LEPTON_LIBS = -L/opt/lepton/leptonSDKEmb32PUB/Debug/ -lLEPTON_SDK
LEPTON_OBJ = /opt/lepton/leptonSDKEmb32PUB/Debug/*.o
LDFLAGS = $(LEPTON_LIBS) $(LEPTON_OBJ)

all: LEPConfigure
