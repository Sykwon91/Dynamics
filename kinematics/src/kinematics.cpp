#include "kinematics.h"

Mat3 position::toRotationMatrix() const
{
    Mat3 R1,R2,R3;
    R1.Rx(this->orientation.x);
    R2.Ry(this->orientation.y);
    R3.Rz(this->orientation.z);
    return R3*R2*R1;
}

position position::ForwardKinematics(const position& child) const
{
    position Forward;
    Forward.frame = this->frame;
    Forward.translation = this->translation + this->toRotationMatrix() * child.translation;
    Forward.orientation = (this->toRotationMatrix() * child.toRotationMatrix()).toEuler();
    return Forward;
}


position position::InvereseKinematics(const position& child) const
{
    //if(this->frame == child.frame) return this;
    position Inverse;
    Inverse.frame = "";
    Inverse.translation =   this->toRotationMatrix().transpose() * (child.translation - this->translation);
    Inverse.orientation = (this->toRotationMatrix().transpose() * child.toRotationMatrix()).toEuler();
    return Inverse;
}

velocity velocity::ForwardKinematics(const velocity& child) const
{
    velocity Forward;
    Forward.translation = this->translation + this->frame_position.toRotationMatrix() * child.translation +   this->toDotRotationMatrix() * this->frame_position.toRotationMatrix() * child.frame_position.translation;

}