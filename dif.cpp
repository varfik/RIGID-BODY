#include <cmath>
#include <iostream>
#include "dif.h"
#include "render.h"

// derivative f(X(t), t) = dX/dt in point t.
RigidBody f_rigidbody(const RigidBody &rb, const Context &context, double time)
{
    RigidBody dt; // result

    // r' = l / M (velocity)
    dt.r = rb.l * context.M_inv;
    // transform quaternion to rotation matrix (I(t))
    dmat3 R(rb.q);

    // omega = I(t) * L = R * I_body * R^(-1) * L
    dvec3 omega = R * context.I_inv * glm::transpose(R) * rb.L;
    // q' = 1/2 * omega * q
    dt.q = 0.5 * dquat(0, omega) * rb.q;

    // define is body in water
    bool is_in_water = rb.r.y - SIZE <= 0; // water line starts in y = 0

    // gravity computing
    dvec3 gravity = dvec3(0, -9.81 * (1.0 / context.M_inv), 0);

    dvec3 buoyancy = dvec3(0.0);
    dvec3 drag = dvec3(0.0);
    double height = 0.0;
    double displaced_fluid = 0.0;
    if (is_in_water) {
        if (isSphere) {
            height = std::max(0.0, -rb.r.y + SIZE); // height of the submerged part of the body
            height = std::min(SIZE * 1.0, height);
            displaced_fluid = M_PI * height * height * (3 * SIZE - height) / 3.0; // amount of displaced fluid
        } else {
            double height = std::max(0.0, -rb.r.y + SIZE);
            displaced_fluid = pow(SIZE, 2) * std::min(height, SIZE * 1.0);
        }
        // buoyancy computing
        buoyancy = dvec3(0.0, 1000 * 9.81 * displaced_fluid, 0.0);
        // drag computing
        drag = -50.0 * dt.r * glm::length(dt.r);
    }
    else {
        drag = -10.0 * dt.r * glm::length(dt.r);
    }
    // total Force
    dvec3 Force = gravity + buoyancy + drag;
    dt.l = Force;

    // center of mass of displaced fluid
    dvec3 buoyancy_center = dvec3(0.0, -height / 2, 0.0);
    // Archimedes' moment of force
    dvec3 Torque = glm::cross(buoyancy_center, buoyancy);

    // total Torque
    dt.L = Torque;
    return dt;
}

// num * rb
RigidBody MulRB(const RigidBody &rb, double num)
{
    RigidBody res;

    res.r = rb.r * num;
    res.q = rb.q * num;
    res.l = rb.l * num;
    res.L = rb.L * num;

    return res;
}

// rb1 + rb2
RigidBody SumRB(const RigidBody &r1, const RigidBody &r2)
{
    RigidBody res;

    res.r = r1.r + r2.r;
    res.q = r1.q + r2.q;
    res.l = r1.l + r2.l;
    res.L = r1.L + r2.L;

    return res;
}


double SolveRungeKutta(RigidBody &rb, const Context &context, double h, double cur_time)
{
    RigidBody k1 = f_rigidbody(rb, context, cur_time);
    RigidBody k2 = f_rigidbody(SumRB(rb, MulRB(k1, h / 2)), context, cur_time + h / 2);
    RigidBody k3 = f_rigidbody(SumRB(rb, MulRB(k2, h / 2)), context, cur_time + h / 2);
    RigidBody k4 = f_rigidbody(SumRB(rb, MulRB(k3, h)), context, cur_time + h);

    rb = SumRB(rb, MulRB(SumRB(k1, SumRB(MulRB(k2, 2), SumRB(MulRB(k3, 2), k4))), h / 6));

    dmat3 R = dmat3(glm::normalize(rb.q));
    dvec3 omega = R * context.I_inv * glm::transpose(R) * rb.L;

    return glm::dot(omega, rb.L);
}
    
