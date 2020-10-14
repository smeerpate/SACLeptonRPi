
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>

#include "bcm_host.h"

#define WIDTH	400//360
#define HEIGHT  400//270

#define PRINTINFO 1
//#define PRINTMOREINFO 1
#define WAITTIME 40

#ifndef ALIGN_UP
// Macro to facilitate word allignment.
#define ALIGN_UP(x,y)  ((x + (y)-1) & ~((y)-1))
// How this macro works...
// e.g. with x=12, y=8:
//     (8'b0000_1100 + 8'b0000_0111) & (8'b1111_1000) =>
//     (8'b0000_1111) & (8'b1111_1000) = 8'b0000_1000 = 16d
// 16 is the smallest multiple of 8 to fit 12 in.
#endif

typedef struct
{
    DISPMANX_DISPLAY_HANDLE_T   display;
    DISPMANX_MODEINFO_T         info;
    void                       *image;
    DISPMANX_UPDATE_HANDLE_T    update;
    DISPMANX_RESOURCE_HANDLE_T  resource;
    DISPMANX_ELEMENT_HANDLE_T   element;
    uint32_t                    vc_image_ptr;

} RECT_VARS_T;


static RECT_VARS_T  gRectVars;


static void FillRect(void *image, int bytesPerLine, int w, int h, int val )
{
    int row;
    int col;

    uint32_t* line = (uint32_t*)image;
    for (row = 0; row < h; row++)
    {
        for (col = 0; col < w; col++)
        {
            if (col & 0x2)
                line[col] = val;
            else
                line[col] = 0;
            //line[col] = val;
        }
        #ifdef PRINTMOREINFO
        printf("[info]: Written line nr %i at address 0x%08x.\n", row, line );
        #endif
        line += (bytesPerLine >> 2); // Need to increment with number of DWORDs per line instead of bytes. Getting segmentation fau otherwise.
    }
}

int openSemaphoreSet(key_t key, int numberSems)
{
    int iSemId;
    if (!numberSems)
        return (-1);

    if ((iSemId = semget(key, numberSems, IPC_CREAT | IPC_EXCL | 0660)) == -1)
        return (-1);

    return iSemId;
}

int ipcOpenSegment(key_t key, int iSegSize)
{
    int iSegId;

    // Try to get a segment of shared memory and store its ID.
    iSegId = shmget(key, iSegSize, IPC_CREAT | 0660);

    if (iSegId == -1)
    {
        // A problem occured...
        return -1;
    }
    return iSegSize;
}



int main(void)
{
    RECT_VARS_T* rectVars;
    uint32_t screen = 0;
    int result;
    VC_RECT_T srcRect;
    VC_RECT_T dstRect;
    VC_IMAGE_TYPE_T imgType = VC_IMAGE_TF_RGBA32; // 8 bit for R,G and B = 24 bit
    int width = WIDTH;
    int height = HEIGHT;
    int bytesPerLine;
    uint32_t bytesToAllocate;
    VC_DISPMANX_ALPHA_T alpha = {DISPMANX_FLAGS_ALPHA_FROM_SOURCE | DISPMANX_FLAGS_ALPHA_FIXED_ALL_PIXELS,
                                 200, // Alpha value.
                                 0};

    rectVars = &gRectVars; // Pointer to global struct variable.
    bytesPerLine = ALIGN_UP(width * 4, 32); // Length of a line in bytes, DWORD allignement.
    bytesPerLine = width*4;

    ///// IPC stuff /////
    key_t key;
    key = ftok(".", 's');
    int iSemId = openSemaphoreSet(key, 1);
    printf("[info]: Created a semaphore set with id %d and key %d.\n", iSemId, key);
    int iShmNumBytes = 640 * 480 * 4; // 640x480 RGBA 8888
    int iSegId = ipcOpenSegment(key, iShmNumBytes);
    printf("[info]: Created a segment of shared memory with id %d and key %d.\n", iSegId, key);
    ////////////////////

    bcm_host_init();

    // Open display area.
    #ifdef PRINTINFO
    printf("[info]: Opening screen nr %i.\n", screen);
    #endif
    rectVars->display = vc_dispmanx_display_open(screen);
    result = vc_dispmanx_display_get_info(rectVars->display, &rectVars->info);
    assert(result == 0);
    #ifdef PRINTINFO
    printf("[info]: Display resolution is %d x %d.\n", rectVars->info.width, rectVars->info.height);
    #endif

    // Allocate memory for the image buffer.
    bytesToAllocate = bytesPerLine * height;
    #ifdef PRINTINFO
    printf("[info]: Rectangle will be %d pixels by %d pixels.\n", width, height);
    printf("[info]: One line counts %d bytes of memory.\n", bytesPerLine);
    printf("[info]: Allocating %d bytes of memory...\n", bytesToAllocate);
    #endif
    rectVars->image = calloc(1, bytesToAllocate);
    assert(rectVars->image != NULL);
    #ifdef PRINTINFO
    printf("[info]: Allocated memory at 0x%08x.\n", (int32_t)rectVars->image);
    #endif

    // Fill the imagebuffer with data.
    FillRect(rectVars->image, bytesPerLine, width, height, 0x0010A0FF); // 0xAARRGGBB

    // Create resources in the video core.
    rectVars->resource = vc_dispmanx_resource_create(imgType, width, height, &rectVars->vc_image_ptr);
    assert(rectVars->resource != 0);
    #ifdef PRINTINFO
    printf("[info]: Created resources at 0x%08x.\n", (int32_t)rectVars->resource);
    #endif

    // Create a destination rectangle
    vc_dispmanx_rect_set(&dstRect, 0, 0, width, height);
    result = vc_dispmanx_resource_write_data(rectVars->resource, imgType, bytesPerLine, rectVars->image, &dstRect);
    assert(result == 0);
    // Create update handle
    rectVars->update = vc_dispmanx_update_start(10);
    assert(rectVars->update);

    // Create source and destinationn rectangles
    vc_dispmanx_rect_set(&srcRect, 0, 0, width << 16, height << 16); // fixed point 16.16??
    vc_dispmanx_rect_set(&dstRect, 300, 300, width, height);

    // Add elements (?)
    rectVars->element = vc_dispmanx_element_add(rectVars->update,
                                                rectVars->display,
                                                2000, // layer nr
                                                &dstRect,
                                                rectVars->resource,
                                                &srcRect,
                                                DISPMANX_PROTECTION_NONE,
                                                &alpha,
                                                NULL, // Clamp
                                                VC_IMAGE_ROT0);

    result = vc_dispmanx_update_submit_sync(rectVars->update);
    assert(result == 0);

    printf("[info]: Sleeping for %d secs...\n", WAITTIME);
    sleep(WAITTIME);

    rectVars->update = vc_dispmanx_update_start(10); // priority 10??
    assert(rectVars->update);


    // Clean up.
    #ifdef PRINTINFO
    printf("[info]: Removing element...\n");
    #endif
    result = vc_dispmanx_element_remove(rectVars->update, rectVars->element);
    assert(result == 0);
    result = vc_dispmanx_update_submit_sync(rectVars->update);
    assert(result == 0);

    #ifdef PRINTINFO
    printf("[info]: Cleaning up allocated memory at 0x%08x.\n", (int32_t)rectVars->image);
    #endif
    free(rectVars->image);

    #ifdef PRINTINFO
    printf("[info]: Deleting video core resources at address 0x%08x.\n", (int32_t)rectVars->resource);
    #endif
    result = vc_dispmanx_resource_delete(rectVars->resource);
    assert(result == 0);

    #ifdef PRINTINFO
    printf("[info]: Closing screen nr %i...\n", screen);
    #endif
    result = vc_dispmanx_display_close(rectVars->display);
    assert(result == 0);

    return 0;
}
