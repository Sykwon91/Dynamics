#pragma once
#include <iostream>
#include <math.h>

struct Vec3 
{
    double x; double y; double z; 
    Vec3 operator+(const Vec3& other) const
    {
        return Vec3{
            this->x + other.x,
            this->y + other.y,
            this->z + other.z
        };
    }
    Vec3 operator-(const Vec3& other) const
    {
        return Vec3{
            this->x - other.x,
            this->y - other.y,
            this->z - other.z
        };
    }
    Vec3 operator*(const double& scale) const
    {
        return Vec3{
            scale * this->x,
            scale * this->y,
            scale * this->z
        };
    }
    friend Vec3 operator*(const double& scale, const Vec3& Vec) 
    {
        return Vec3{
            scale * Vec.x,
            scale * Vec.y,
            scale * Vec.z
        };
    }
    double dot(const Vec3& other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }
    Vec3 cross(const Vec3& other) const
    {
        
        return Vec3{this->y * other.z - this->z * other.y, this->z * other.x - this->x * other.z, this->x * other.y - this->y * other.x};
    }
};

struct Position
{
    Vec3 translation;
    Vec3 rotation;

    void display();
    void set(double x, double y, double z, double roll, double pitch, double yaw);
};

struct Cylinder
{
    double H;
    double R;
    Position Center;
    bool collision;

    void display();
    void checkcollsion(Vec3 position);
    Cylinder(double Radius, double Height, Position position);
};

struct Box
{
    double L;
    double W;
    double H;
    Position Center;
    bool collision;

    void display();
    void checkcollsion(Vec3 position);
    Box(double Long, double Width, double Height, Position position);
};

struct Plane
{
    Vec3 norm;
    Vec3 point;

    void display();
    void checkcollsion(Position position);
    Vec3 closest(Cylinder cyl);
    Plane(Vec3 norm, Vec3 point);
};