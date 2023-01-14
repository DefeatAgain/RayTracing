#pragma once

#include "Primitive.h"

class Sphere :
    public Primitive
{
public:
    Eigen::Vector3f center;
    float radius, radius_square;

    Sphere(const Eigen::Vector3f& c, float r, std::shared_ptr<Material> m);

    Intersection Intersect(const Ray& r) const override;

    Intersection Sample() const override;
};

