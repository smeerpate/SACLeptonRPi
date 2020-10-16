DMX_INC = -I/opt/vc/include -I/opt/vc/include/interface/vmcs_host/ -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux

EGL_INC =
GLES_INC = -ICommon/
INCLUDES = $(DMX_INC) $(EGL_INC) $(GLES_INC)
CFLAGS = $(INCLUDES)

CPPFLAGS = -march=armv7-a -mtune=cortex-a7

DMX_LIBS = -L/opt/vc/lib/ -lbcm_host -lvcos -lvchiq_arm -lpthread
EGL_LIBS = -L/opt/vc/lib -lbrcmEGL -lbrcmGLESv2
ESP_LIBS = -LCommon -lRPes2pg
LDFLAGS = $(DMX_LIBS) $(EGL_LIBS) $(ESP_LIBS) -lm

SRC = SACDisplayArray SACPrintEGLInfo SACCreateEGLWindow SACCreateEGLWindowWithIPC OGLESHelloTriangle OGLESSimpleImageWithIPC

all: $(SRC)

$(SRC): Common/libRPes2pg.a

Common/libRPes2pg.a:
	make -C Common
