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


Mat3 velocity::toAngularVelocitySkew() const
{
    Mat3 Matrix;
    Matrix.mat[0][1] = -this->orientation.z;
    Matrix.mat[0][2] =  this->orientation.y;
    Matrix.mat[1][2] = -this->orientation.x;
    Matrix.mat[1][0] =  this->orientation.z;
    Matrix.mat[2][0] = -this->orientation.y;
    Matrix.mat[2][1] =  this->orientation.x;
    
    return Matrix;
}

position position::InverseKinematics(const position& child) const
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
    Forward.translation = this->translation
                         + this->frame_position.toRotationMatrix() * child.translation 
                         + this->toAngularVelocitySkew() * this->frame_position.toRotationMatrix() * child.frame_position.translation;
    Mat3 DotRotationMatrix = this->frame_position.toRotationMatrix() * child.toAngularVelocitySkew() * this->frame_position.toRotationMatrix().transpose()
                            + this->toAngularVelocitySkew();
    Forward.orientation = DotRotationMatrix.toDotEuler();
    return Forward;
}

velocity velocity::InverseKinematics(const velocity& child) const
{
    velocity Inverse;
    Inverse.frame_position = this->frame_position.InverseKinematics(child.frame_position);
    Inverse.orientation = (this->frame_position.toRotationMatrix().transpose() * (child.toAngularVelocitySkew() * child.frame_position.toRotationMatrix() - this->toAngularVelocitySkew() * this->frame_position.toRotationMatrix() * Inverse.frame_position.toRotationMatrix() ) * Inverse.frame_position.toRotationMatrix().transpose()).toDotEuler() ;
    Inverse.translation = this->frame_position.toRotationMatrix().transpose() *(child.translation
                         - this->translation
                         - this->toAngularVelocitySkew() * this->frame_position.toRotationMatrix() * Inverse.frame_position.translation);
    return Inverse;
}

Mat3 acceleration::toAngularAccelerationSkew() const
{
    Mat3 Matrix;
    Matrix.mat[0][1] = -this->orientation.z;
    Matrix.mat[0][2] =  this->orientation.y;
    Matrix.mat[1][2] = -this->orientation.x;
    Matrix.mat[1][0] =  this->orientation.z;
    Matrix.mat[2][0] = -this->orientation.y;
    Matrix.mat[2][1] =  this->orientation.x;
    
    return Matrix;
}


acceleration acceleration::ForwardKinematics() const
{
    
}