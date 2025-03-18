#ifndef __DIF_H
#define __DIF_H

#define SIZE 20

#include <glm/glm.hpp>
#include <glm/ext.hpp>

using namespace glm;

// struct for constants
struct Context
{
    // 1 / mass
    double M_inv;
    // I_body^(-1)
    dmat3 I_inv;
    double side;
};

struct RigidBody
{
    dvec3 r, l, L; // position, impulse, moment of impulse
    dquat q; // rotation quaternion
};

double SolveEuler(RigidBody &rb, const Context &context, double h, double cur_time);

double SolveRungeKutta(RigidBody &rb, const Context &context, double h, double cur_time);

#endif // __DIF_H