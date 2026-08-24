/*
    Inertial Measurement Unit Maths Library
    Copyright (C) 2013-2014  Samuel Cowen
	www.camelsoftware.com

    Bug fixes and cleanups by Gé Vissers (gvissers@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef IMUMATH_QUATERNION_HPP
#define IMUMATH_QUATERNION_HPP

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <Eigen/Dense>
#include <QQuaternion>
#include <QVector3D>
#include "matrix.h"

#define DEGTORAD 0.0174532925199432957
#define RADTODEG 57.295779513082320876
#define HALFPI 1.5707963267948966192313216916398
#define PI 3.1415926535897932384626433832795
#define TWOPI 6.283185307179586476925286766559


namespace imu
{

typedef struct AxisAngle
{
  float x = 0;
  float y = 0;
  float z = 0;
  float angle = 0;
};

class Quaternion
{   

public:
    Quaternion(): _w(1.0), _x(0.0), _y(0.0), _z(0.0) {}

    Quaternion(double w, double x, double y, double z):
        _w(w), _x(x), _y(y), _z(z) {}

    Quaternion(double w, Vector<3> vec):
        _w(w), _x(vec.x()), _y(vec.y()), _z(vec.z()) {}

    double& w()
    {
        return _w;
    }
    double& x()
    {
        return _x;
    }
    double& y()
    {
        return _y;
    }
    double& z()
    {
        return _z;
    }

    double w() const
    {
        return _w;
    }
    double x() const
    {
        return _x;
    }
    double y() const
    {
        return _y;
    }
    double z() const
    {
        return _z;
    }

    void Set(double w, double x, double y, double z)
    {
      _w = w;  _x = x;  _y = y;  _z = z;
    }

    //magnitude
    // -parameters : none
    // -return value : _Tp
    // -when called on a quaternion object q, returns the magnitude q
    double magnitude() const
    {
        return sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
    }

    //conjugate
    // -parameters : none
    // -return value : quaternion
    // -when called on a quaternion object q, returns the conjugate of q
    Quaternion conjugate() const
    {
        return Quaternion(_w, -_x, -_y, -_z);
    }

    Eigen::Vector3d Quaternion_rotate(Quaternion q, Eigen::Vector3d v)
       {
           Eigen::Vector3d output;

           //assert(output != NULL);

           float ww = q.w() * q.w();
           float xx = q.x() * q.x();
           float yy = q.y() * q.y();
           float zz = q.z() * q.z();
           float wx = q.w() * q.x();
           float wy = q.w() * q.y();
           float wz = q.w() * q.z();
           float xy = q.x() * q.y();
           float xz = q.x() * q.z();
           float yz = q.y() * q.z();

           // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
           // p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x;
           // p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y;
           // p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z;

           output.x() = (ww*v[0] + 2*wy*v[2] - 2*wz*v[1] + xx*v[0] + 2*xy*v[1] + 2*xz*v[2] - zz*v[0] - yy*v[0]);

           output.y() = (2*xy*v[0] + yy*v[1] + 2*yz*v[2] + 2*wz*v[0] - zz*v[1] + ww*v[1] - 2*wx*v[2] - xx*v[1]);

           output.z() = (2*xz*v[0] + 2*yz*v[1] + zz*v[2] - 2*wy*v[0] - yy*v[2] + 2*wx*v[1] - xx*v[2] + ww*v[2]);

           return output;
       }

    Quaternion integral_quternion (double x, double y, double z, double dt)
    {
        // Integrate rate of change of quaternion
        _w +=( -_x  * x  -  _y   *  y - _z   *  z ) * (0.5 * dt);
        _x +=(  _w  * x  +  _y   *  z - _z   *  y ) * (0.5 * dt);
        _y +=(  _w  * y  -  _x   *  z + _z   *  x ) * (0.5 * dt);
        _z +=(  _w  * z  +  _x   *  y - _y   *  x ) * (0.5 * dt);

        double norm = sqrt(_w * _w + _x * _x + _y * _y + _z * _z);

        if (norm != 0.0) {
           norm = 1.0 / norm;
           _w = _w * norm;
           _x = _x * norm;
           _y = _y * norm;
           _z = _z * norm;

           return (*this);
         }

      return (*this);
    }

    Quaternion integral_quternion (Eigen::Vector3d r, double dt)
    {
        // Integrate rate of change of quaternion
        _w +=( -_x  * r[1]  -  _y   *  r[0] - _z   *  r[2] ) * (0.5 * dt);
        _x +=(  _w  * r[1]  +  _y   *  r[2] - _z   *  r[0] ) * (0.5 * dt);
        _y +=(  _w  * r[0]  -  _x   *  r[2] + _z   *  r[1] ) * (0.5 * dt);
        _z +=(  _w  * r[2]  +  _x   *  r[0] - _y   *  r[1] ) * (0.5 * dt);

        double norm = sqrt(_w * _w + _x * _x + _y * _y + _z * _z);

        if (norm != 0.0) {
           norm = 1.0 / norm;
           _w = _w * norm;
           _x = _x * norm;
           _y = _y * norm;
           _z = _z * norm;

           return (*this);
         }

        return (*this);
    }

    Quaternion Euler_Sempling (Quaternion q, float dt)
    {
        // Integrate rate of change of quaternion
       _w += q.w() * (0.5f * dt);
       _x += q.x() * (0.5f * dt);
       _y += q.y() * (0.5f * dt);
       _z += q.z() * (0.5f * dt);

      this->Normalize_1();

      return (*this);

    }

    QQuaternion Quaternion_to_QQuaternion_for_OGL ()
    {
     return QQuaternion {(float)-_w, (float)_x, (float)_y, (float)_z};
    }


    void fromAxisAngle(const Vector<3>& axis, double theta)
    {
        _w = cos(theta/2);
        //only need to calculate sine of half theta once
        double sht = sin(theta/2);
        _x = axis.x() * sht;
        _y = axis.y() * sht;
        _z = axis.z() * sht;
    }

    void fromAxisAngle(AxisAngle input)
    {
        _w = cos(input.angle/2);
        //only need to calculate sine of half theta once
        double sht = sin(input.angle/2);
        _x = input.x * sht;
        _y = input.y * sht;
        _z = input.z * sht;
    }

    void itegrate_fromAxisAngle(AxisAngle input, float dt)
    {
        _w += cos((input.angle * dt)/2);
        //only need to calculate sine of half theta once
        double sht = sin((input.angle * dt)/2);
        _x += input.x * sht;
        _y += input.y * sht;
        _z += input.z * sht;
    }

    void fromMatrix(const Matrix<3>& m)
    {
        double tr = m.trace();

        double S;
        if (tr > 0)
        {
            S = sqrt(tr+1.0) * 2;
            _w = 0.25 * S;
            _x = (m(2, 1) - m(1, 2)) / S;
            _y = (m(0, 2) - m(2, 0)) / S;
            _z = (m(1, 0) - m(0, 1)) / S;
        }
        else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2))
        {
            S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
            _w = (m(2, 1) - m(1, 2)) / S;
            _x = 0.25 * S;
            _y = (m(0, 1) + m(1, 0)) / S;
            _z = (m(0, 2) + m(2, 0)) / S;
        }
        else if (m(1, 1) > m(2, 2))
        {
            S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
            _w = (m(0, 2) - m(2, 0)) / S;
            _x = (m(0, 1) + m(1, 0)) / S;
            _y = 0.25 * S;
            _z = (m(1, 2) + m(2, 1)) / S;
        }
        else
        {
            S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
            _w = (m(1, 0) - m(0, 1)) / S;
            _x = (m(0, 2) + m(2, 0)) / S;
            _y = (m(1, 2) + m(2, 1)) / S;
            _z = 0.25 * S;
        }
    }

    void toAxisAngle(Vector<3>& axis, double& angle) const
    {
        double sqw = sqrt(1-_w*_w);
        if (sqw == 0) //it's a singularity and divide by zero, avoid
            return;

        angle = 2 * acos(_w);
        axis.x() = _x / sqw;
        axis.y() = _y / sqw;
        axis.z() = _z / sqw;
    }

    AxisAngle toAxisAngle() const
    {
        AxisAngle output;

        double sqw = sqrt(1-_w*_w);

        if (sqw != 0){ //it's a singularity and divide by zero, avoid

        output.angle = 2 * acos(_w);
        output.x = _x / sqw;
        output.y = _y / sqw;
        output.z = _z / sqw;
        }

      return output;
    }

    Matrix<3> toMatrix() const
    {
        Matrix<3> ret;
        ret.cell(0, 0) = 1 - 2*_y*_y - 2*_z*_z;
        ret.cell(0, 1) = 2*_x*_y - 2*_w*_z;
        ret.cell(0, 2) = 2*_x*_z + 2*_w*_y;

        ret.cell(1, 0) = 2*_x*_y + 2*_w*_z;
        ret.cell(1, 1) = 1 - 2*_x*_x - 2*_z*_z;
        ret.cell(1, 2) = 2*_y*_z - 2*_w*_x;

        ret.cell(2, 0) = 2*_x*_z - 2*_w*_y;
        ret.cell(2, 1) = 2*_y*_z + 2*_w*_x;
        ret.cell(2, 2) = 1 - 2*_x*_x - 2*_y*_y;
        return ret;
    }


    // Returns euler angles that represent the quaternion.  Angles are
    // returned in rotation order and right-handed about the specified
    // axes:
    //
    //   v[0] is applied 1st about z (ie, roll)
    //   v[1] is applied 2nd about y (ie, pitch)
    //   v[2] is applied 3rd about x (ie, yaw)
    //
    // Note that this means result.x() is not a rotation about x;
    // similarly for result.z().
    //

    //Более надёжная
    Vector<3> ToEulerAngles (Quaternion q) {

        Vector<3> angles;

        // yaw (z-axis rotation)
        double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        angles.z() = std::atan2(sinr_cosp, cosr_cosp) * RADTODEG;

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1)
            angles.y() = std::copysign(M_PI / 2, sinp) * RADTODEG; // use 90 degrees if out of range
        else
            angles.y() = std::asin(sinp) * RADTODEG;

        // roll (x-axis rotation)
        double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        angles.x() = std::atan2(siny_cosp, cosy_cosp) * RADTODEG;

        return angles;
    }

    // Более быстрая но могут проблемы с шарним замком
    Vector<3> ToEuler() const
    {
        Vector<3> ret;
        double sqw = _w*_w;
        double sqx = _x*_x;
        double sqy = _y*_y;
        double sqz = _z*_z;

        double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
        double test = _x *_y + _z *_w;

            if (test > 0.499*unit) { // singularity at north pole
                ret.z() = 2 * atan2(_x,_w);
                ret.x() = HALFPI;
                ret.y() = 0;
                return ret;
            }
            if (test < -0.499*unit) { // singularity at south pole
                ret.z() = -2 * atan2(_x,_w);
                ret.x() = -HALFPI;
                ret.y() = 0;
                return ret;
            }

        ret.y() = atan2(2.0*(_x*_w+_y*_z),(sqx-sqy-sqz+sqw)) * RADTODEG;
        ret.x() = asin(-2.0*(_x*_y-_z*_w)/(sqx+sqy+sqz+sqw)) * RADTODEG;
        ret.z() = atan2(2.0*(_y*_w+_x*_z),(-sqx-sqy+sqz+sqw)) * RADTODEG;

        return ret;
    }

    void EulerToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
    {
        pitch *= DEGTORAD;
        roll *= DEGTORAD;
        yaw *= DEGTORAD;

        // Abbreviations for the various angular functions
        double cosYaw = cos(-yaw * 0.5);
        double sinYaw = sin(-yaw * 0.5);
        double cosPitch = cos(pitch * 0.5);
        double sinPitch = sin(pitch * 0.5);
        double cosRoll = cos(roll * 0.5);
        double sinRoll = sin(roll * 0.5);

//        _w = c1 * c2 * c3 - s1 * s2 * s3;
//        _x = s1 * s2 * c3 + c1 * c2 * s3;
//        _y = s1 * c2 * c3 + c1 * s2 * s3;
//        _z = c1 * s2 * c3 - s1 * c2 * s3;

        _w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
        _x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
        _y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
        _z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    }

    Vector<3> toAngularVelocity(double dt) const
    {
        Vector<3> ret;
        Quaternion one(1.0, 0.0, 0.0, 0.0);
        Quaternion delta = one - *this;
        Quaternion r = (delta/dt);
        r = r * 2;
        r = r * one;

        ret.x() = r.x();
        ret.y() = r.y();
        ret.z() = r.z();
        return ret;
    }

    Vector<3> AngularVelocity_to_Quaternion (Quaternion q, double dt) const
    {
        Vector<3> ret;
        Quaternion delta = q - *this;
        Quaternion r = (delta/dt);
        r = r * 2;
        r = r * q;

        ret.x() = r.x();
        ret.y() = r.y();
        ret.z() = r.z();
        return ret;
    }

    Vector<3> rotateVector(const Vector<2>& v) const
    {
        return rotateVector(Vector<3>(v.x(), v.y()));
    }

    Vector<3> rotateVector(const Vector<3>& v) const
    {
        Vector<3> qv(_x, _y, _z);
        Vector<3> t = qv.cross(v) * 2.0;
        return v + t*_w + qv.cross(t);
    }


    Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(
            _w*q._w - _x*q._x - _y*q._y - _z*q._z,
            _w*q._x + _x*q._w + _y*q._z - _z*q._y,
            _w*q._y - _x*q._z + _y*q._w + _z*q._x,
            _w*q._z + _x*q._y - _y*q._x + _z*q._w
        );
    }

    Quaternion operator *= (const Quaternion  q)
    {
       _w = _w*q._w - _x*q._x - _y*q._y - _z*q._z;
       _x = _w*q._x + _x*q._w + _y*q._z - _z*q._y;
       _y = _w*q._y + _y*q._w + _z*q._x - _x*q._z;
       _z = _w*q._z + _z*q._w + _x*q._y - _y*q._x;

       return (*this);
    }

    Quaternion operator+(const Quaternion& q) const
    {
        return Quaternion(_w + q._w, _x + q._x, _y + q._y, _z + q._z);
    }

    Quaternion operator-(const Quaternion& q) const
    {
        return Quaternion(_w - q._w, _x - q._x, _y - q._y, _z - q._z);
    }

    Quaternion operator/(double scalar) const
    {
        return Quaternion(_w / scalar, _x / scalar, _y / scalar, _z / scalar);
    }


    Quaternion operator += (const Quaternion q)
    {
      _w += q._w;
      _x += q._x;
      _y += q._y;
      _z += q._z;

      return (*this);
    }


    Quaternion operator -= (const Quaternion q)
    {
      _w -= q._w;
      _x -= q._x;
      _y -= q._y;
      _z -= q._z;

      return (*this);
    }


    Quaternion operator*(double scalar) const
    {
        return scale(scalar);
    }


    //scale
    // -parameters :  s- a value to scale q1 by
    // -return value: quaternion
    // -returns the original quaternion with each part, w,x,y,z, multiplied by some scalar s
    Quaternion scale(double scalar) const
    {
        return Quaternion(_w * scalar, _x * scalar, _y * scalar, _z * scalar);
    }

    bool operator== (Quaternion q) const
    {
      return ((_w == q._w) && (_x == q._x) &&
              (_y == q._y) && (_z == q._z));
    }

    bool operator!= (Quaternion q) const
    {
      return ((_w != q._w) || (_x != q._x) ||
              (_y != q._y) || (_z != q._z));
    }


    Quaternion operator= (Quaternion q)
    {
      _w = q._w;
      _x = q._x;
      _y = q._y;
      _z = q._z;

      return *this;
    }

     Quaternion operator= (double w)
    {
      _w = w;
      _x = 0;
      _y = 0;
      _z = 0;

      return *this;
    }


     //norm
     // -parameters : none
     // -return value : _Tp
     // -when called on a quaternion object q, returns the norm of q
      double Norm2()
     {
       return (_w *_w + _x *_x + _y *_y + _z *_z);
     }


     // q.Normalize() scales q such that it is unit size
     Quaternion Normalize_1()
     {
       double invNorm;
       invNorm = (double)1.0 / (double)sqrt(Norm2());

       _w *= invNorm;
       _x *= invNorm;
       _y *= invNorm;
       _z *= invNorm;

       return *this;
     }

     void Normalize_2()
     {
         double mag = magnitude();
         *this = this->scale(1/mag);
     }

     Quaternion operator/ (Quaternion q2) const
     {
       // compute invQ2 = q2^{-1}
       Quaternion invQ2;
       double invNorm2 = 1.0 / q2.Norm2();
       invQ2._w =  q2._w * invNorm2;
       invQ2._x = -q2._x * invNorm2;
       invQ2._y = -q2._y * invNorm2;
       invQ2._z = -q2._z * invNorm2;

       // result = *this * invQ2
       return (*this * invQ2);

     }

     Quaternion operator /= (Quaternion q)
     {
       (*this) = (*this)*q.Inverse();
       return (*this);
     }


     // -parameters : none
     // -return value : quaternion
     // -when called on a quaternion object q, returns the inverse of
     Quaternion Inverse()
     {
       return conjugate().scale(1/Norm2());
     }


     //UnitQuaternion
     // -parameters : none
     // -return value : quaternion
     // -when called on quaterion q, takes q and returns the unit quaternion of q
     Quaternion UnitQuaternion() const
     {
       return (*this).scale(1/(*this).magnitude());
     }


     // -parameters : vector
     // -return value : void
     // -when given a 3D vector, v, rotates v by this quaternion
     void QuatRotation(Vector<3> v)
     {
       Quaternion qv(0, v[0], v[1], v[2]);
       Quaternion qm = (*this) * qv * (*this).Inverse();

       v[0] = qm._x;
       v[1] = qm._y;
       v[2] = qm._z;
     }

      void GetRotation(double * angle, double unitAxis[3])
     {
       if ((_w >= ((double)1)) || (_w <= (double)(-1)))
       {
         // identity; this check is necessary to avoid problems with acos if s is 1 + eps
         *angle = 0;
         unitAxis[0] = 1;
         unitAxis[0] = 0;
         unitAxis[0] = 0;
         return;
       }

       *angle = 2.0 * acos(_w);
       double sin2 = _x*_x + _y*_y + _z*_z; //sin^2(*angle / 2.0)

       if (sin2 == 0)
       {
         // identity rotation; angle is zero, any axis is equally good
         unitAxis[0] = 1;
         unitAxis[0] = 0;
         unitAxis[0] = 0;
       }
       else
       {
         double inv = 1.0 / sqrt(sin2); // note: *angle / 2.0 is on [0,pi], so sin(*angle / 2.0) >= 0, and therefore the sign of sqrt can be safely taken positive
         unitAxis[0] = _x * inv;
         unitAxis[1] = _y * inv;
         unitAxis[2] = _z * inv;
       }
     }

private:

    double _w, _x, _y, _z;
};

}

#endif
