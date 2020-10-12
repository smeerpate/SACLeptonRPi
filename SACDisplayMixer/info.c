#include <stdio.h>
#include <assert.h>
#include <bcm_host.h>

typedef struct
{
	uint32_t screenWidth;
	uint32_t screenHeight;
	DISPMANX_DISPLAY_HANDLE_T dispManDisplay;
} DISPMANX_STATE_T;

DISPMANX_STATE_T state;
DISPMANX_STATE_T* pState = &state;

void dispmanx_init(DISPMANX_STATE_T* state)
{
	int32_t succes = 0;

	bcm_host_init();

	DISPMANX_ELEMENT_HANDLE_T dispManElement;
	DISPMANX_UPDATE_HANDLE_T dispManUpdate;
	VC_RECT_T dstRect;
	VC_RECT_T srcRect;

	succes = graphics_get_display_size(0, &state->screenWidth, &state->screenHeight);
	assert(succes >= 0);
	printf("Screen height = %d, screen width = %d\n", state->screenHeight, state->screenWidth);
}

int main(int argc, char* argv[])
{
	dispmanx_init(pState);
	assert(vc_dispmanx_display_close(pState->dispManDisplay) == 0);
	return 0;
}
