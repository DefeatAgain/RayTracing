#pragma once

#include <Eigen/Eigen>

#include "Primitive.h"

class Triangle :
    public Primitive
{
public:
    Eigen::Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Eigen::Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Eigen::Vector2f t0, t1, t2; // texture coords
    Eigen::Vector3f normal;
public:
    Triangle(const Eigen::Vector3f& _v0, const Eigen::Vector3f& _v1, const Eigen::Vector3f& _v2,
        std::shared_ptr<Material> mat);

    ~Triangle() {}
    
    Intersection Intersect(const Ray& r) const override;
    
    Intersection Sample() const override;

    void Reset();
};