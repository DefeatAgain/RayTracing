#pragma once

#include<Eigen/Eigen>
#include<vector>

#include "Ray.h"
#include "Intersection.h"
#include "BoundingBox.h"

class Primitive;

class BVHTree
{
public:
	struct BVHNode
	{
		BoundingBox box;
		std::vector<std::shared_ptr<Primitive>> pris;
		std::shared_ptr<BVHNode> left;
		std::shared_ptr<BVHNode> right;
	};

	BVHTree(std::vector<std::shared_ptr<Primitive>>& pris);
	~BVHTree();

	size_t CalculateNodeNum();

	Intersection Intersect(const Ray& r);
	Intersection IntersectImpl(std::shared_ptr<BVHNode> node, const Ray& r);
private:
	std::shared_ptr<BVHNode> RecursiveBuild(std::vector<std::shared_ptr<Primitive>> pris);
	void CalculateNode(size_t& sz, std::shared_ptr<BVHNode> node);
private:
	std::shared_ptr<BVHNode> root;
};