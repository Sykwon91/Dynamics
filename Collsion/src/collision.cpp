#include "collision.h"

void Position::display()
{
    std::cout << translation.x << std::endl;
    std::cout << translation.y << std::endl;
    std::cout << translation.z << std::endl;
    std::cout << rotation.x << std::endl;
    std::cout << rotation.y << std::endl;
    std::cout << rotation.z << std::endl;
}

void Position::set(double x, double y, double z, double roll, double pitch, double yaw)
{
    translation.x  = x;
    translation.y = y;
    translation.z = z;
    rotation.x = roll;
    rotation.y = pitch;
    rotation.z = yaw;
}

void Cylinder::display()
{
    std::cout << "Radius : " << R << std::endl;
    std::cout << "Height : " << H << std::endl;
    std::cout << "Center : " << std::endl;
    Center.display();
}

void Cylinder::checkcollsion(Position position)
{
    collision = false;

    Vec3 dxyz;
    dxyz.x = position.translation.x - Center.translation.x;
    dxyz.y = position.translation.y - Center.translation.y;
    dxyz.z = position.translation.z - Center.translation.z;

    double cosx = cos(Center.rotation.x);
    double cosy = cos(Center.rotation.y);
    double cosz = cos(Center.rotation.z);
    double sinx = sin(Center.rotation.x);
    double siny = sin(Center.rotation.y);
    double sinz = sin(Center.rotation.z);

    const double dx = (cosz * cosy) * dxyz.x + (cosx * siny) * dxyz.y + (-siny) * dxyz.z;
    const double dy = (cosz * siny * sinx - sinz * cosx) * dxyz.x + (sinz * siny * sinx + cosz * cosx) * dxyz.y + (sinx * cosy) * dxyz.z;
    const double dz = (cosz * siny * cosx + sinz * sinx) * dxyz.x + (sinz * siny * cosx - cosz * sinx) * dxyz.y + (cosx * cosy) * dxyz.z;

    //std::cout << dx << std::endl;
    //std::cout << dy << std::endl;
    //std::cout << dz << std::endl;
    if (dx*dx + dz*dz < this->R * this->R)
    {
        if (this->H/2 < dy ) collision = false;
        else if (-this->H/2 > dy ) collision = false;
        else collision = true;
    }
}

Cylinder::Cylinder(double Radius, double Height, Position position)
{
    this->R = Radius;
    this->H = Height;
    this->Center = position;
    this->collision = false;
}


void Box::display()
{
    std::cout << "Long : " << L << std::endl;
    std::cout << "Width : " << W << std::endl;
    std::cout << "Height : " << H << std::endl;
    std::cout << "Center : " << std::endl;
    Center.display();
}


void Box::checkcollsion(Position position)
{
    collision = false;

    Vec3 dxyz;
    dxyz.x = position.translation.x - Center.translation.x;
    dxyz.y = position.translation.y - Center.translation.y;
    dxyz.z = position.translation.z - Center.translation.z;

    double cosx = cos(Center.rotation.x);
    double cosy = cos(Center.rotation.y);
    double cosz = cos(Center.rotation.z);
    double sinx = sin(Center.rotation.x);
    double siny = sin(Center.rotation.y);
    double sinz = sin(Center.rotation.z);

    const double dx = (cosz * cosy) * dxyz.x + (cosx * siny) * dxyz.y + (-siny) * dxyz.z;
    const double dy = (cosz * siny * sinx - sinz * cosx) * dxyz.x + (sinz * siny * sinx + cosz * cosx) * dxyz.y + (sinx * cosy) * dxyz.z;
    const double dz = (cosz * siny * cosx + sinz * sinx) * dxyz.x + (sinz * siny * cosx - cosz * sinx) * dxyz.y + (cosx * cosy) * dxyz.z;

    //std::cout << dx << std::endl;
    //std::cout << dy << std::endl;
    //std::cout << dz << std::endl;
    if (abs(dx) < this->L/2.f)
    {
        if(abs(dy) < this->W/2.f)
        {
            if (abs(dz) < H/2 ) collision = true;
        }
    }
}

Box::Box(double Long, double Width, double Height, Position position)
{
    this->L = Long;
    this->W = Width;
    this->H = Height;
    this->Center = position;
    this->collision = false;
}