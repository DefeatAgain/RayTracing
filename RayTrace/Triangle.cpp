#include "Triangle.h"
#include "math.h"

Triangle::Triangle(const Eigen::Vector3f& _v0, const Eigen::Vector3f& _v1, const Eigen::Vector3f& _v2,
    std::shared_ptr<Material> mat)
    :Primitive(), v0(_v0), v1(_v1), v2(_v2)
{
    e1 = v1 - v0;
    e2 = v2 - v0;
    normal = e1.cross(e2).normalized();
    area = e1.cross(e2).norm() * 0.5f;

    SetBoundingBox(BoundingBox(v0, v1, v2));

    SetMaterial(mat);
}

Intersection Triangle::Intersect(const Ray& r) const
{
    Intersection inter;
    if (r.dir.dot(normal) > 0.0f) // ray must intersec in front surface
        return inter;

   /* float u, v, t_tmp = 0.0f;
    Eigen::Vector3f pvec = r.dir.cross(e2);
    float det = e1.dot(pvec);
    if (std::fabs(det) < math::EPSILON)
        return inter;

    double det_inv = 1.0f / det;
    Eigen::Vector3f tvec = r.ori - v0;
    u = tvec.dot(pvec) * det_inv;
    if (u < 0.0f || u > 1.0f)
        return inter;
    Eigen::Vector3f qvec = tvec.cross(e1);
    v = r.dir.dot(qvec) * det_inv;
    if (v < 0.0f || u + v > 1.0f)
        return inter;
    t_tmp = e2.dot(qvec) * det_inv;*/

    auto[t, croods] = math::RayIntersectTriangle0(r.ori, r.dir, v0, e1, e2);
    if (t < 0.0f)
        return inter;

    inter.happened = true;
    inter.coords = croods.x() * v0 + croods.y() * v1 + croods.z() * v2;
    inter.distance = t;
    inter.emited = material->emited;
    inter.mat = material;
    inter.normal = normal;
    inter.prim = (Primitive*)(this);
    inter.aim = "Triangle";

    return inter;
}

Intersection Triangle::Sample() const
{
    Intersection inter;
    float x = std::sqrt(math::GetRandomFloat());
    float y = math::GetRandomFloat();
    inter.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
    inter.normal = normal;
    inter.mat = material;
    inter.happened = true;
    return inter;
}

void Triangle::Reset()
{
    e1 = v1 - v0;
    e2 = v2 - v0;
    normal = e1.cross(e2).normalized();
    area = e1.cross(e2).norm() * 0.5f;

    SetBoundingBox(BoundingBox(v0, v1, v2));
}
