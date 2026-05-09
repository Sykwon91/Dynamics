#ifndef WRENCH_H
#define WRENCH_H
#include "linearalgebra.h"
struct Wrench
{
    public:
        Vec3 Force;
        Vec3 Torque;
        Vec3 PointOfApplication;
    private:
        void operator+=(const Wrench& other)
        {
            this->Force += other.Force;
            this->Torque += other.Torque;
        }
        void operator-=(const Wrench& other)
        {
            this->Force -= other.Force;
            this->Torque -= other.Torque;
        }
        void operator*=(double scale)
        {
            this->Force *= scale;
            this->Torque *= scale;
        }
        Wrench operator+(const Wrench& other) const
        {
            return Wrench{this->Force + other.Force, this->Torque + other.Torque};
        }
        Wrench operator-(const Wrench& other) const
        {
            return Wrench{this->Force - other.Force, this->Torque - other.Torque};
        }
};
#endif