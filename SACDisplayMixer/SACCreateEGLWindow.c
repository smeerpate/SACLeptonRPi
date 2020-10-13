#include <stdio.h>
#include <assert.h>
#include <math.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>

#include <bcm_host.h>

typedef struct
{
    EGLDisplay display;
    EGLSurface surface;
    EGLContext context;
    EGLConfig config;
    EGL_DISPMANX_WINDOW_T nativeWindow;
    DISPMANX_DISPLAY_HANDLE_T dispmanDisplay;
} EGL_STATE_T;

EGL_STATE_T eglState = {.display = EGL_NO_DISPLAY,
                        .surface = EGL_NO_SURFACE,
                        .context = EGL_NO_CONTEXT};
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

    bcm_host_init();

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
    printf("[info]: Got EGL context.");
}

void initDispmanx(EGL_STATE_T* state)
{
    EGL_DISPMANX_WINDOW_T* nativeWindow = &pEglState->nativeWindow;
    int32_t iResult = 0;
    uint32_t screenWidth;
    uint32_t screenHeight;

    DISPMANX_ELEMENT_HANDLE_T dispmanElement;
    DISPMANX_DISPLAY_HANDLE_T dispmanDisplay;
    DISPMANX_UPDATE_HANDLE_T dispmanUpdate;
    VC_RECT_T dstRect;
    VC_RECT_T srcRect;

    bcm_host_init();

    // Create an EGL window surface.
    iResult = graphics_get_display_size(0, &screenWidth, &screenHeight);
    assert(iResult >= 0);
    dstRect.x = 0;
    dstRect.y = 0;
    dstRect.width = screenWidth;
    dstRect.height = screenHeight;
    srcRect.x = 0;
    srcRect.y = 0;
    srcRect.width = screenWidth << 16; // Fixed point notation in 32bit DWORD 16.16.
    srcRect.height = screenHeight << 16; // Also fixed point notation in 32bit DWORD 16.16.

    dispmanDisplay = vc_dispmanx_display_open(0);
    dispmanUpdate = vc_dispmanx_update_start(0); // Priority = 0
    state->dispmanDisplay = dispmanDisplay;
    dispmanElement = vc_dispmanx_element_add(dispmanUpdate,
                                             dispmanDisplay,
                                             0, // layer 0
                                             &dstRect,
                                             0, // resource (DISPMANX_RESOURCE_HANDLE_T)
                                             &srcRect,
                                             DISPMANX_PROTECTION_NONE,
                                             0, // Alpha
                                             0, // Clamp
                                             0 // Transform
                                             );

    // Build an EGL_DISPMANX_WINDOW_T from the dispmanx window.
    nativeWindow->element = dispmanElement;
    nativeWindow->width = screenWidth;
    nativeWindow->height = screenHeight;
    vc_dispmanx_update_submit_sync(dispmanUpdate);
    iResult = vc_dispmanx_element_remove(dispmanUpdate, dispmanElement);
    assert(iResult == 0);
    printf("[info]: Got Dispmanx window.");
}

void eglFromDispmanx(EGL_STATE_T* state)
{
    EGLBoolean bResult;
    state->surface = eglCreateWindowSurface(state->display, state->config, &pEglState->nativeWindow, NULL);
    assert(state->surface != EGL_NO_SURFACE);

    // Now connect the context to the surface.
    bResult = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
    assert(EGL_FALSE != bResult);
}

void cleanup(int s)
{
    if (pEglState->surface != EGL_NO_SURFACE && eglDestroySurface(pEglState->display, pEglState->surface))
        printf("[info]: Surface destroyed ok.\n");
    if (pEglState->context != EGL_NO_CONTEXT && eglDestroyContext(pEglState->display, pEglState->context))
        printf("[info]: Context destroyed ok.\n");
    if (pEglState->display != EGL_NO_DISPLAY && eglTerminate(pEglState->display))
        printf("[info]: Display terminated ok.\n");
    if (eglReleaseThread())
        printf("[info]: EGL thread resources released ok.\n");
    if (vc_dispmanx_display_close(pEglState->dispmanDisplay) == 0)
        printf("[info]: Dispmanx display released ok.\n");
    bcm_host_deinit();
    exit(s);
}

int main(int argc, char* argv[])
{
    signal(SIGINT, cleanup);

    initEGL(pEglState);
    initDispmanx(pEglState);

    eglFromDispmanx(pEglState);

    glClearColor(0.05, 1.0, 0.7, 0.5);
    glClear(GL_COLOR_BUFFER_BIT);
    glFlush();

    eglSwapBuffers(pEglState->display, pEglState->surface);

    sleep(30);

    cleanup(0);
    return 0;
}
