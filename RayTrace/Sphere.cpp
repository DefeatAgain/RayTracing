#include "Sphere.h"
#include "math.h"

Sphere::Sphere(const Eigen::Vector3f& c, float r, std::shared_ptr<Material> m):
    center(c), radius(r), radius_square(r* r)
{
    area = 4.0f * math::PI * radius_square;

    SetBoundingBox(BoundingBox(center.array() - radius,
                                center.array() + radius));

    SetMaterial(m);
}

Intersection Sphere::Intersect(const Ray& r) const
{
    Intersection inter;
    Eigen::Vector3f L = r.ori - center;
    float a = r.dir.dot(r.dir);
    float b = r.dir.dot(L);
    float c = L.dot(L) - radius_square;
    auto t0 = math::SolveQuadratic(a, b, c);
    if (t0 < 0.0f)
        return inter;

    inter.happened = true;
    inter.coords = r.ori + r.dir * t0;
    inter.normal = (inter.coords - center).normalized();
    inter.mat = material;
    inter.prim = (Primitive*)this;
    inter.distance = t0;
    inter.aim = "Sphere";

    return inter;
}

Intersection Sphere::Sample() const
{
    Intersection inter;
    float theta = 2.0f * math::PI * math::GetRandomFloat();
    float phi = math::PI * math::GetRandomFloat();
    Eigen::Vector3f dir(std::cos(phi), std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta));
    inter.coords = center + radius * dir;
    inter.normal = dir;
    return inter;
}
