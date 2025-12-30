#pragma once
#include <iostream>
#include <math.h>

struct Vec3 { double x; double y; double z; };

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
    void checkcollsion(Position position);
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
    void checkcollsion(Position position);
    Box(double Long, double Width, double Height, Position position);
};