#ifndef __RENDER_H
#define __RENDER_H

#include <GL/glut.h>

#include "dif.h"

// show global variables to other files
extern Context context;
extern RigidBody rb;
extern bool isSphere;

void Run(int argc, char *argv[]);
void GraphEnergy();

#endif //__RENDER_H
