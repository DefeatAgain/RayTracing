#pragma once
#include <Eigen/Eigen>

class Material;
class Primitive;

struct Intersection
{
    bool happened;
    bool front_happend;
    Eigen::Vector3f coords;
    Eigen::Vector2f tcoords;
    Eigen::Vector3f normal;
    bool emited;
    std::shared_ptr<Material> mat;
    Primitive* prim;
    float distance;

    std::string aim;

    Intersection() :
        happened(false),
        front_happend(false),
        emited(false),
        mat(nullptr),
        prim(nullptr),
        distance(std::numeric_limits<float>::max())
    {}
};