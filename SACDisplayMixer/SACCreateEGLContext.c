#include <stdio.h>
#include <assert.h>
#include <math.h>

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

void initEGL(EGL_STATE_T* state)
{
    EGLBoolean bResult;
    EGLint numConfigs;
    EGLConfig* configs;
    static const EGLint attributeList[] = {EGL_RED_SIZE, 8,
                                           EGL_GREEN_SIZE, 8,
                                           EGL_BLUE_SIZE, 8,
                                           EGL_ALPHA_SIZE, 8,
                                           EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
                                           EGL_NONE};
    static const EGLint contextAttributes[] = {EGL_CONTEXT_CLIENT_VERSION, 2,
                                               EGL_NONE};

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

    /////// Choose a configuration ////////
    // Get the number of configurations.
    eglGetConfigs(state->display, NULL, 0, &numConfigs);
    printf("EGL has %d configurations.\n", numConfigs);
    // Allocate memory for all configurations.
    configs = calloc(numConfigs, sizeof *configs);
    // Read all configurations in "configs".
    eglGetConfigs(state->display, configs, numConfigs, &numConfigs);
    // Get the first available configuration.
    bResult = eglChooseConfig(state->display, attributeList, &state->config, 1, &numConfigs);
    assert(EGL_FALSE != bResult);

    // Choose the OpenGL ES API
    bResult = eglBindAPI(EGL_OPENGL_ES_API);
    assert(EGL_FALSE != bResult);

    // Now create the rendering context.
    state->context = eglCreateContext(state->display, state->config, EGL_NO_CONTEXT, contextAttributes);
    assert(EGL_NO_CONTEXT != state->context);
    printf("[info]: Got EGL context.\n");
}

int main(int argc, char* argv[])
{
    initEGL(pEglState);
    return 0;
}
