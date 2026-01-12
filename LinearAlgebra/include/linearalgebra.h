#ifndef LinearAlgebra_H
#define LinearAlgebra_H

#include <cmath>

struct Vec3
{
    double x{0.0}, y{0.0}, z{0.0};

    Vec3 operator+(const Vec3& other) const
    {
        return { x + other.x, y + other.y, z + other.z };
    }

    Vec3 operator-(const Vec3& other) const
    {
        return { x - other.x, y - other.y, z - other.z };
    }

    Vec3 operator*(double scale) const
    {
        return { x * scale, y * scale, z * scale };
    }

    friend Vec3 operator*(double scale, const Vec3& v)
    {
        return { v.x * scale, v.y * scale, v.z * scale };
    }

    double dot(const Vec3& other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    Vec3 cross(const Vec3& other) const
    {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    double norm() const
    {
        return std::sqrt(dot(*this));
    }

    Vec3 normalized(double eps = 1e-12) const
    {
        const double n = norm();
        if (n < eps) return {0.0, 0.0, 0.0};
        return (*this) * (1.0 / n);
    }
};

struct Mat3
{
    // row-major: mat[row][col]
    double mat[3][3]{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    static Mat3 Identity()
    {
        Mat3 I;
        I.mat[0][0] = 1.0;
        I.mat[1][1] = 1.0;
        I.mat[2][2] = 1.0;
        return I;
    }

    Mat3 operator*(double scale) const
    {
        Mat3 out;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                out.mat[r][c] = mat[r][c] * scale;
        return out;
    }

    friend Mat3 operator*(double scale, const Mat3& m)
    {
        return m * scale;
    }

    Vec3 operator*(const Vec3& v) const
    {
        return {
            mat[0][0] * v.x + mat[0][1] * v.y + mat[0][2] * v.z,
            mat[1][0] * v.x + mat[1][1] * v.y + mat[1][2] * v.z,
            mat[2][0] * v.x + mat[2][1] * v.y + mat[2][2] * v.z
        };
    }

    Mat3 operator*(const Mat3& other) const
    {
        Mat3 out;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                out.mat[r][c] =
                    mat[r][0] * other.mat[0][c] +
                    mat[r][1] * other.mat[1][c] +
                    mat[r][2] * other.mat[2][c];
            }
        }
        return out;
    }
    Mat3 operator+(const Mat3& other) const
    {
        Mat3 out;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                out.mat[r][c] = this->mat[r][c] + other.mat[r][c];
            }
        }
        return out;
    }

    Mat3 transpose() const
    {
        Mat3 out;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                out.mat[r][c] = mat[c][r];
        return out;
    }
};

#endif // LinearAlgebra_H
