LIBES = libRPes2pg.a

DMX_INC = -I/opt/vc/include/ -I/opt/vc/include/interface/vmcs_host/ -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux

EGL_INC =
GLES_INC =
INCLUDES = $(DMX_INC) $(EGL_INC) $(GLES_INC)
CFLAGS = $(INCLUDES) -DRPI_NO_X -DPI_DISPLAY=4

LIBOBJ = esUtil.o esShader.o esShapes.o esTransform.o

$(LIBES): $(LIBOBJ)
	ar rc $(LIBES) $(LIBOBJ)
	ranlib $(LIBES)

$(LIBOBJ): Makefile
clean:
	rm -f $(LIBOBJ) $(LIBES)
