#pragma once
#include <memory>

#include "Intersection.h"
#include "BoundingBox.h"
#include "Ray.h"
#include "Material.h"

class Primitive
{
public:
	Primitive() : area(0.0f) {}
	~Primitive() {}

	virtual Intersection Intersect(const Ray& r) const = 0;

	virtual Intersection Sample() const = 0;

	void SetBoundingBox(const BoundingBox& b) { box = b; };

	void SetMaterial(std::shared_ptr<Material> m) { material = m; }
public:
	 std::shared_ptr<Material> material;
	 BoundingBox box;
	 float area;
};