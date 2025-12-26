#include <iostream>
struct Vec3
{
    double x;
    double y;
    double z;
};

struct Position
{
    Vec3 translation;
    Vec3 rotation;
    void display()
    {
        std::cout << this->translation.x << std::endl;
        std::cout << this->translation.y << std::endl;
        std::cout << this->translation.z << std::endl;
        std::cout << this->rotation.x << std::endl;
        std::cout << this->rotation.y << std::endl;
        std::cout << this->rotation.z << std::endl;
    };
    void set(double x, double y, double z, double roll, double pitch, double yaw)
    {
        this->translation.x  = x;
        this->translation.y = y;
        this->translation.z = z;
        this->rotation.x = roll;
        this->rotation.y = pitch;
        this->rotation.z = yaw;
    }
};

struct Cylinder
{
    double H;
    double R;
    Position Center;
    bool collision;
    void display()
    {
        std::cout << "Radius : " <<  this->R << std::endl;
        std::cout << "Height : " <<  this->H << std::endl;
        std::cout << "Center : " << std::endl;
        this->Center.display();
    };
    void checkcollsion(Position position)
    {
        if( (this->Center.translation.x - position.translation.x) * 
            (this->Center.translation.x - position.translation.x) +
            (this->Center.translation.z - position.translation.z) *
            (this->Center.translation.z - position.translation.z)
            < this->R * this->R
        )
        {
            if( position.translation.y < this->Center.translation.y - this->H/2)
            {
                this->collision = false;
            }
            else if(position.translation.y > this->Center.translation.y + this->H/2)
            {
                this->collision = false;
            }
            else{this->collision = true;}
        }
    }
    Cylinder(double Radius, double Height, Position position)
    {
        this->R = Radius;
        this->H = Height;
        this->Center = position;
        this->collision = false;
    };
};

int main()
{
    Position point,pointc;
    point.set(1,0,0,0,0,0);
    pointc.set(100,0,0,0,0,0);
    Cylinder C1(1,1,point);

    C1.display();
    C1.checkcollsion(pointc);
    if(C1.collision){std::cout << "collsion" <<std::endl;}
    else{std::cout << "no collision"<<std::endl;}

    return 0;
}