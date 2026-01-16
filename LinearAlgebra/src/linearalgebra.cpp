#include "linearalgebra.h"

    Vec3 Vec3::operator+(const Vec3& other) const
    {
        return { x + other.x, y + other.y, z + other.z };
    }

    Vec3 Vec3::operator-(const Vec3& other) const
    {
        return { x - other.x, y - other.y, z - other.z };
    }

    Vec3 Vec3::operator*(double scale) const
    {
        return { x * scale, y * scale, z * scale };
    }

    Vec3 operator*(double scale, const Vec3& v)
    {
        return { v.x * scale, v.y * scale, v.z * scale };
    }

    double Vec3::dot(const Vec3& other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    Vec3 Vec3::cross(const Vec3& other) const
    {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    double Vec3::norm() const
    {
        return std::sqrt(dot(*this));
    }

    Vec3 Vec3::normalized(double eps = 1e-12) const
    {
        const double n = norm();
        if (n < eps) return {0.0, 0.0, 0.0};
        return (*this) * (1.0 / n);
    }


    void Mat3::Identity()
    {
        
        this->mat[0][0] = 1.0;
        this->mat[1][1] = 1.0;
        this->mat[2][2] = 1.0;
    }

    void Mat3::Rx(double roll) 
    {
        double cr = cos(roll);
        double sr = sin(roll);
        this->mat[0][0] = 1.0;
        this->mat[1][1] = cr;
        this->mat[1][2] = -sr;
        this->mat[2][1] = sr;
        this->mat[2][2] = cr;
    }
    void Mat3::Ry(double pitch) 
    {
        double cp = cos(pitch);
        double sp = sin(pitch);
        this->mat[0][0] = cp;
        this->mat[0][2] = sp;
        this->mat[1][1] = 1.0;
        this->mat[2][0] = -sp;
        this->mat[2][2] = cp;
    }

    void Mat3::Rz(double yaw) 
    {
        double cy = cos(yaw);
        double sy = sin(yaw);
        this->mat[0][0] = cy;
        this->mat[0][1] = -sy;
        this->mat[1][0] = sy;
        this->mat[1][1] = cy;
        this->mat[2][2] = 1.0;
    }

    Vec3 Mat3::toEuler() const
    {
        Vec3 e;
        if (std::abs(this->mat[0][2]) < 1.0) 
        {
            e.y = std::asin(-this->mat[0][2]);
            if(std::cos(e.y) > 0) e.x  = std::atan2(this->mat[2][1], this->mat[2][2]);
            else  e.x  = std::atan2(-this->mat[2][1],-this->mat[2][2]);
            if(std::cos(e.y) > 0) e.z   = std::atan2(this->mat[1][0], this->mat[0][0]);
            else{e.z   = std::atan2(-this->mat[1][0], -this->mat[0][0]);}
            
        }
        else
        {
            // Gimbal lock
            e.y = (this->mat[0][2] <= -1.0) ? -M_PI / 2.0 : M_PI / 2.0;
            e.x  = 0.0;
            if(std::cos(e.y) > 0) e.z   = std::atan2(this->mat[1][0], this->mat[0][0]);
            else{e.z   = std::atan2(-this->mat[1][0], -this->mat[0][0]);}
        }
    
        return e;
    }

    Vec3 Mat3::toDotEuler() const
    {
        return Vec3{-this->mat[1][2],this->mat[0][2],-this->mat[0][1]};
    }

    Mat3 Mat3::operator*(double scale) const
    {
        Mat3 out;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                out.mat[r][c] = mat[r][c] * scale;
        return out;
    }

    Mat3 operator*(double scale, const Mat3& m)
    {
        return m * scale;
    }

    Vec3 Mat3::operator*(const Vec3& v) const
    {
        return {
            mat[0][0] * v.x + mat[0][1] * v.y + mat[0][2] * v.z,
            mat[1][0] * v.x + mat[1][1] * v.y + mat[1][2] * v.z,
            mat[2][0] * v.x + mat[2][1] * v.y + mat[2][2] * v.z
        };
    }

    Mat3 Mat3::operator*(const Mat3& other) const
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
    Mat3 Mat3::operator+(const Mat3& other) const
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

    Mat3 Mat3::operator-(const Mat3& other) const
    {
        Mat3 out;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                out.mat[r][c] = this->mat[r][c] - other.mat[r][c];
            }
        }
        return out;
    }

    Mat3 Mat3::transpose() const
    {
        Mat3 out;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                out.mat[r][c] = mat[c][r];
        return out;
    }