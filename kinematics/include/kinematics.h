#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "linearalgebra.h"
#include <string>

struct position
{
    std::string frame{};
    std::string childframe{};
    Vec3 translation;
    Vec3 orientation;
    Mat3 toRotationMatrix() const;
    position ForwardKinematics(const position& child) const;
    position InverseKinematics(const position& child) const;
};

struct velocity
{
    position frame_position;
    Vec3 translation;
    Vec3 orientation;
    
    Mat3 toAngularVelocitySkew() const;
    velocity ForwardKinematics(const velocity& child) const;
    velocity InverseKinematics(const velocity& child) const;
    void update(const position& frame);
};

struct acceleration
{
    velocity frame_velocity;
    Vec3 translation;
    Vec3 orientation;
    Mat3 toAngularAccelerationSkew() const;
    acceleration ForwardKinematics(const acceleration& child) const;
    acceleration InverseKinematics(const acceleration& child) const;
    void update();
    void update(const velocity& frame);
};

#endif