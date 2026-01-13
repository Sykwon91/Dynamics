#ifndef LinearAlgebra_H
#define LinearAlgebra_H

#include <cmath>

struct Vec3
{
    double x{0.0}, y{0.0}, z{0.0};

    Vec3 operator+(const Vec3& other) const;

    Vec3 operator-(const Vec3& other) const;

    Vec3 operator*(double scale) const;

    friend Vec3 operator*(double scale, const Vec3& v);

    double dot(const Vec3& other) const;

    Vec3 cross(const Vec3& other) const;

    double norm() const;

    Vec3 normalized(double eps) const;
};

struct Mat3
{
    // row-major: mat[row][col]
    double mat[3][3]{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    void Identity();

    void Rx(double roll);
    
    void Ry(double pitch);

    void Rz(double yaw);

    Mat3 operator*(double scale) const;

    friend Mat3 operator*(double scale, const Mat3& m);

    Vec3 operator*(const Vec3& v) const;

    Mat3 operator*(const Mat3& other) const;

    Mat3 operator+(const Mat3& other) const;

    Mat3 transpose() const;
};

#endif // LinearAlgebra_H
