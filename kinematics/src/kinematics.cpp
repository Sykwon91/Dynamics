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
    Forward.frame_position = this->frame_position.ForwardKinematics(child.frame_position);
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
    
    Inverse.orientation = (this->frame_position.toRotationMatrix().transpose() * (child.toAngularVelocitySkew() - this->toAngularVelocitySkew()) * this->frame_position.toRotationMatrix()).toDotEuler() ;
    Inverse.translation = this->frame_position.toRotationMatrix().transpose() * ( this->toAngularVelocitySkew().transpose() * (child.frame_position.translation - this->frame_position.translation) 
                         + child.translation - this->translation);
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


acceleration acceleration::ForwardKinematics(const acceleration& child) const
{
    acceleration Forward;
    Forward.frame_velocity = this->frame_velocity.ForwardKinematics(child.frame_velocity);
    Forward.translation = this->translation
                         + this->toAngularAccelerationSkew() * this->frame_velocity.frame_position.toRotationMatrix() * child.frame_velocity.frame_position.translation 
                         + this->frame_velocity.toAngularVelocitySkew() * this->frame_velocity.toAngularVelocitySkew() * this->frame_velocity.frame_position.toRotationMatrix() * child.frame_velocity.frame_position.translation
                         + 2 * this->frame_velocity.toAngularVelocitySkew() * this->frame_velocity.frame_position.toRotationMatrix() * child.frame_velocity.translation
                         + this->frame_velocity.frame_position.toRotationMatrix() * child.translation;
    
    Mat3 DDotRotationMatrix = this->toAngularAccelerationSkew()
                             + this->frame_velocity.toAngularVelocitySkew() * this->frame_velocity.frame_position.toRotationMatrix() * child.frame_velocity.toAngularVelocitySkew() * this->frame_velocity.frame_position.toRotationMatrix().transpose()
                             + this->frame_velocity.frame_position.toRotationMatrix() * child.toAngularAccelerationSkew() * this->frame_velocity.frame_position.toRotationMatrix().transpose()
                             + this->frame_velocity.frame_position.toRotationMatrix()  * child.frame_velocity.toAngularVelocitySkew() * this->frame_velocity.frame_position.toRotationMatrix().transpose() * this->frame_velocity.toAngularVelocitySkew().transpose();
    Forward.orientation = DDotRotationMatrix.toDotEuler();
    return Forward;   
}

acceleration acceleration::InverseKinematics(const acceleration& child) const
{
    acceleration Inverse;
    Inverse.frame_velocity = this->frame_velocity.InverseKinematics(child.frame_velocity);
    Inverse.orientation = (this->frame_velocity.frame_position.toRotationMatrix().transpose() * (child.toAngularAccelerationSkew() - this ->toAngularAccelerationSkew()) * this->frame_velocity.frame_position.toRotationMatrix().transpose()).toDotEuler();
    Inverse.translation = this->frame_velocity.frame_position.toRotationMatrix().transpose() * (child.translation -  this->translation 
                        - this->frame_velocity.toAngularVelocitySkew() * this->frame_velocity.toAngularVelocitySkew() * this->frame_velocity.frame_position.toRotationMatrix() * Inverse.frame_velocity.frame_position.translation
                        - this->toAngularAccelerationSkew() * this->frame_velocity.frame_position.toRotationMatrix() * Inverse.frame_velocity.frame_position.translation
                        - 2.f * this->frame_velocity.toAngularVelocitySkew() * this->frame_velocity.frame_position.toRotationMatrix()  * Inverse.frame_velocity.translation
                        );
    return Inverse;
}