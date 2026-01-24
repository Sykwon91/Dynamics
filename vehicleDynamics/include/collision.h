#pragma once
#include "linearalgebra.h"
#include <iostream>
#include <math.h>

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
    Vec3 closest(Box box);
    Plane(Vec3 norm, Vec3 point);
};