#include "linearalgebra.h"
#include <string>

struct position
{
    std::string frame{}; 
    Vec3 translation;
    Vec3 orientation;
    Mat3 toRotationMatrix() const;
    position ForwardKinematics(const position& child) const;
    position InvereseKinematics(const position& child) const;
};

struct velocity
{
    std::string frame{};
    Vec3 translation;
    Vec3 orientation;
    position frame_position;
    Mat3 toDotRotationMatrix() const;
    velocity ForwardKinematics(const velocity& child) const;
    velocity InvereseKinematics(const velocity& child) const;
};

struct acceleration
{
    std::string frame{};
    Vec3 translation;
    Vec3 orientation;
    position frame_position;
    velocity frame_velocity;
    Mat3 toDDotRotationMatrix() const;
    acceleration ForwardKinematics() const;
    acceleration InvereseKinematics() const;
};
