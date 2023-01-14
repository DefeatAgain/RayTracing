#pragma once
#include <vector>
#include <memory>

#include "Intersection.h"

class Primitive;


class Light
{
public:
    enum class LightType
    {
        TRIANGLE
    };
public:
    Light(const std::vector<std::shared_ptr<Primitive>>& pris, LightType lt_);
    ~Light() {}

    Intersection Sample(float& pdf);
public:
    std::vector<std::shared_ptr<Primitive>> pris;
    float area;
    LightType lt;
};

