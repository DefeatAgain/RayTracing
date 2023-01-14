#pragma once

#include <vector>
#include <memory>

#include "BVH.h"
#include "Ray.h"
#include "Light.h"

class Primitive;
class Triangle;
class BVHTree;

const float MIX_PDF_PROBABILITY = 0.5f;

class Scene
{
public:
	Scene() {}
	~Scene() {}

	void AddObject(const std::vector<std::shared_ptr<Primitive>>& tirs);
	void AddObject(std::shared_ptr<Primitive> pri) { pris.emplace_back(pri); }
	void AddLight(const std::vector<Light>& li);


	void BuildBVHTree(); 

	size_t BVHNodeNum() { return tree->CalculateNodeNum(); }

	Eigen::Vector3f CastRay(const Ray& r, int depth);
private:
	Intersection Intersect(const Ray& r);
	Intersection SampleLight(float& pdf);

	std::vector<std::shared_ptr<Primitive>> pris;
	std::vector<Light> lights;
	std::shared_ptr<BVHTree> tree;

	float light_area;
};

