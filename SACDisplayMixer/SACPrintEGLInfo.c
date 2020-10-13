/*
 * Code stolen from Jan Newmarch's "Raspberry Pi GPU Audio Video
 * Programming" book chapter 5 p.28.
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>

typedef struct
{
    uint32_t scrnWidth;
    uint32_t scrnHeight;
    EGLDisplay display;
    EGLSurface surface;
    EGLContext context;
    EGLConfig config;
} EGL_STATE_T;

EGL_STATE_T eglState;
EGL_STATE_T* pEglState = &eglState;

void printConfigInfo(int configNr, EGLDisplay display, EGLConfig* config)
{
    int iTemp;

    printf("Configuration %d is:\n", configNr);

    eglGetConfigAttrib(display, *config, EGL_RED_SIZE, &iTemp);
    printf("\tRed size is %d: \n", iTemp);
    eglGetConfigAttrib(display, *config, EGL_BLUE_SIZE, &iTemp);
    printf("\tBlue size is %d: \n", iTemp);
    eglGetConfigAttrib(display, *config, EGL_GREEN_SIZE, &iTemp);
    printf("\tGreen size is %d: \n", iTemp);

    eglGetConfigAttrib(display, *config, EGL_BUFFER_SIZE, &iTemp);
    printf("\tBuffer size is %d: \n", iTemp);

    eglGetConfigAttrib(display, *config, EGL_BIND_TO_TEXTURE_RGB, &iTemp);
    if (iTemp == EGL_TRUE)
        printf("\tCan be bound to RGB texture.\n");
    else
        printf("\tCan't be bound to RGB texture.\n");

    eglGetConfigAttrib(display, *config, EGL_BIND_TO_TEXTURE_RGBA, &iTemp);
    if (iTemp == EGL_TRUE)
        printf("\tCan be bound to RGBA texture.\n");
    else
        printf("\tCan't be bound to RGBA texture.\n");
}

void initEGL(EGL_STATE_T* state)
{
    EGLBoolean bResult;
    EGLint numConfigs;
    EGLConfig* configs;

    // Get an EGL display connection.
    state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (state->display == EGL_NO_DISPLAY)
    {
        fprintf(stderr, "No display connection matching native_display available.\n");
    }

    // Initialize the EGL display connection.
    bResult = eglInitialize(state->display, NULL, NULL);
    if (bResult == EGL_FALSE)
    {
        fprintf(stderr, "Can't initialize EGL!\n");
    }

    // Get the number of configurations.
    eglGetConfigs(state->display, NULL, 0, &numConfigs);
    printf("EGL has %d configurations.\n", numConfigs);

    // Allocate memory for the configurations.
    configs = calloc(numConfigs, sizeof *configs);
    eglGetConfigs(state->display, configs, numConfigs, &numConfigs);

    // Get all config info.
    int iConfCnt;
    for (iConfCnt = 0; iConfCnt < numConfigs; iConfCnt++)
    {
        printConfigInfo(iConfCnt, state->display, &configs[iConfCnt]);
    }
}

int main(int argc, char* argv[])
{
    initEGL(pEglState);
    return 0;
}
