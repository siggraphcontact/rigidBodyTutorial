#pragma once

#include <Eigen/Dense>

// List of geometry type ids.
enum eGeometryType { kSphere, kBox };

// Generic geometry interface.
//
class Geometry
{
public:

    virtual ~Geometry() {}

    virtual Eigen::Matrix3f computeInertia(float _mass) = 0;

    virtual eGeometryType getType() const  = 0;

protected:
    Eigen::Matrix3f m_I;          // Inertia 3x3 matrix for this. Only used for local computations. (internal)
};


// Sphere geometry.
//
class Sphere : public Geometry
{
public:
    float radius;           // Sphere radius.

    Sphere(float _radius) : radius(_radius) {}
    virtual ~Sphere() {}

    virtual Eigen::Matrix3f computeInertia(float _mass) override
    {
        m_I.setZero();
        m_I(0,0) = m_I(1,1) = m_I(2,2) = (2.0f/5.0f) * _mass * radius * radius;
        return m_I;
    }

    virtual eGeometryType getType() const override { return kSphere; }

};

// Box geometry.
//
class Box : public Geometry
{
public:
    Eigen::Vector3f dim;        // Box dimensions.

    Box(const Eigen::Vector3f& _dim) : dim(_dim) {

    }
    virtual ~Box() {}

    virtual Eigen::Matrix3f computeInertia(float _mass) override
    {
        m_I.setZero();
        m_I(0,0) = (1.0f/12.0f)*_mass*(dim[1]*dim[1] + dim[2]*dim[2]);
        m_I(1,1) = (1.0f/12.0f)*_mass*(dim[0]*dim[0] + dim[2]*dim[2]);
        m_I(2,2) = (1.0f/12.0f)*_mass*(dim[0]*dim[0] + dim[1]*dim[1]);
        return m_I;
    }

    virtual eGeometryType getType() const override { return kBox; }

};
