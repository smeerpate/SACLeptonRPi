// Authors: Aaftab Munshi, Dan Ginsburg, Dave Shreiner
// Copyright: the authors and Jan Newmarch
// Simple_Image.c
//
// This is a simple example that draws an image with a 2D
// texture image.
//
#include <stdlib.h>
#include <stdio.h>
#include "esUtil.h"
#include <sys/ipc.h>
#include <sys/shm.h>

/////////////////// Global variables ///////////////////
typedef struct {
   // Handle to a program object
   GLuint programObject;
   // Attribute locations
   GLint positionLoc;
   GLint texCoordLoc;
   // Sampler location
   GLint samplerLoc;
   // Texture handle
   GLuint textureId;
   GLubyte *image;
   int width, height;
} UserData;

char* pSharedMemory;
////////////////////////////////////////////////////////

///
// IPC related functions
//
int ipcOpenSegment(key_t key, int iSegSize)
{
    int iSegId;
    // Try to get a segment of shared memory and store its ID.
    // Permission 0660: User can read and write, Group can read and write. Both can not exec.
    iSegId = shmget(key, iSegSize, IPC_CREAT | 0666);
    printf("[INFO]: Created a segment of shared memory with id %d and key %d.\n", iSegId, key);
    if (iSegId == -1)
    {
        // A problem occured...
        return -1;
    }
    return iSegId;
}

char* ipcAttachToSegment(int iSegId)
{
    return(shmat(iSegId, 0, 0)); // Let the kernel find some memory block + no flags.
}

int ipcDetachFromSegmentWithPointer(char* pSegPointer)
{
    // Signal the kernel that we don't longer need the shared memory segment.
    return(shmdt(pSegPointer));
}

char* initSharedMemory(int iNumBytes)
{
    char* pSegStart;
    key_t key;
    key = ftok("/home/pi/SACLeptonRPi", 'i');
    int iSegId = ipcOpenSegment(key, iNumBytes);
    pSegStart = ipcAttachToSegment(iSegId);
    return pSegStart;
}

///
// Create a simple 2x2 texture image with four different colors
//
GLuint CreateSimpleTexture2D(ESContext *esContext) {
   // Texture object handle
   GLuint textureId;
   UserData *userData = esContext->userData;

   GLubyte *pixels = userData->image;
   userData->width = esContext->width;
   userData->height = esContext->height;
   // Use tightly packed data
   glPixelStorei ( GL_UNPACK_ALIGNMENT, 1 );
   // Generate a texture object
   glGenTextures ( 1, &textureId );
   // Bind the texture object
   glBindTexture ( GL_TEXTURE_2D, textureId );
   // Load the texture
   glTexImage2D ( GL_TEXTURE_2D, 0, GL_RGB,
		  userData->width, userData->height,
		  0, GL_RGB, GL_UNSIGNED_BYTE, pixels );
   // Set the filtering mode
   glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
   glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
   return textureId;
}

///
// Initialize the shader and program object
//
int Init ( ESContext *esContext ) {
    UserData *userData = esContext->userData;
    GLbyte vShaderStr[] =
      "attribute vec4 a_position; \n"
      "attribute vec2 a_texCoord; \n"
      "varying vec2 v_texCoord; \n"
      "void main() \n"
      "{ \n"
      " gl_Position = a_position; \n"
      " v_texCoord = a_texCoord; \n"
      "} \n";

    GLbyte fShaderStr[] =
      "precision mediump float; \n"
      "varying vec2 v_texCoord; \n"
      "uniform sampler2D s_texture; \n"
      "void main() \n"
      "{ \n"
      " gl_FragColor = texture2D( s_texture, v_texCoord );\n"
      "} \n";
   // Load the shaders and get a linked program object
   userData->programObject = esLoadProgram ( vShaderStr, fShaderStr );
   // Get the attribute locations
   userData->positionLoc = glGetAttribLocation ( userData->programObject, "a_position" );
   userData->texCoordLoc = glGetAttribLocation ( userData->programObject, "a_texCoord" );

   // Get the sampler location
   userData->samplerLoc = glGetUniformLocation ( userData->programObject, "s_texture" );
   // Load the texture
   userData->textureId = CreateSimpleTexture2D (esContext);
   glClearColor ( 0.0f, 0.0f, 0.0f, 0.0f );
   return GL_TRUE;
}

///
// Draw a triangle using the shader pair created in Init()
//
void Draw ( ESContext *esContext ) {
   UserData *userData = esContext->userData;
   GLfloat vVertices[] = { -1.0f, 1.0f, 0.0f, // Position 0
                            0.0f, 1.0f, // TexCoord 0
                           -1.0f, -1.0f, 0.0f, // Position 1
                            0.0f, 0.0f, // TexCoord 1
                            1.0f, -1.0f, 0.0f, // Position 2
                            1.0f, 0.0f, // TexCoord 2
                            1.0f, 1.0f, 0.0f, // Position 3
                            1.0f, 1.0f // TexCoord 3
                         };
   GLushort indices[] = { 0, 1, 2, 0, 2, 3 };

   // Set the viewport
   glViewport ( 0, 0, esContext->width, esContext->height );

   // Clear the color buffer
   glClear ( GL_COLOR_BUFFER_BIT );
   // Use the program object
   glUseProgram ( userData->programObject );
   // Load the vertex position
   glVertexAttribPointer ( userData->positionLoc, 3, GL_FLOAT,
                           GL_FALSE, 5 * sizeof(GLfloat), vVertices );
   // Load the texture coordinate
   glVertexAttribPointer ( userData->texCoordLoc, 2, GL_FLOAT,
                           GL_FALSE, 5 * sizeof(GLfloat), &vVertices[3] );
   glEnableVertexAttribArray ( userData->positionLoc );
   glEnableVertexAttribArray ( userData->texCoordLoc );
   // Bind the texture
   glActiveTexture ( GL_TEXTURE0 );
   glBindTexture ( GL_TEXTURE_2D, userData->textureId );
   // Set the sampler texture unit to 0
   glUniform1i ( userData->samplerLoc, 0 );
   glDrawElements ( GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, indices );
}

void Update(ESContext *esContext, float deltatime)
{
   UserData *userData = esContext->userData;
   //userData->image = pSharedMemory;
   //userData->textureId = CreateSimpleTexture2D (esContext);
   //esContext->userData = userData;
   // Load the texture
   glTexImage2D ( GL_TEXTURE_2D, 0, GL_RGB,
                  userData->width, userData->height,
                  0, GL_RGB, GL_UNSIGNED_BYTE, pSharedMemory );

   //printf("shmValue = %d\n", *pSharedMemory);
   return;
}

///
// Cleanup
//
void ShutDown ( ESContext *esContext ) {
   UserData *userData = esContext->userData;
   // Delete texture object
   glDeleteTextures ( 1, &userData->textureId );
   // Delete program object
   glDeleteProgram ( userData->programObject );

   free(esContext->userData);

   // Detach from shared memory.
   ipcDetachFromSegmentWithPointer(pSharedMemory);
}


int main ( int argc, char *argv[] ) {
   ESContext esContext;
   UserData userData;
   int width, height;
   GLubyte *image;

   // The size of our image. If needed bigger change display width/height in esUtil as well.
   width = 640;
   height = 480;

   pSharedMemory = initSharedMemory(width * height * 3);
   image = pSharedMemory;

   userData.image = image;
   userData.width = width;
   userData.height = height;

   esInitContext ( &esContext );
   esContext.userData = &userData;
   esCreateWindow ( &esContext, "Simple Texture 2D", width, height, ES_WINDOW_RGB );

   if ( !Init ( &esContext ) )
      return 0;

   esRegisterDrawFunc ( &esContext, Draw );
   esRegisterUpdateFunc(&esContext, Update);

   esMainLoop ( &esContext );
   ShutDown ( &esContext );
}
