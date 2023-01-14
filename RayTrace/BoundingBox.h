#pragma once
#include <Eigen/Eigen>

#include "Ray.h"
#include "math.h"

enum class Axis
{
	X,
	Y,
	Z
};

class BoundingBox
{
	friend BoundingBox Union(const BoundingBox& b1, const BoundingBox& b2)
	{
		return BoundingBox(Eigen::Vector3f(std::fmin(b1.min_pos.x(), b2.min_pos.x()),
							std::fmin(b1.min_pos.y(), b2.min_pos.y()),
							std::fmin(b1.min_pos.z(), b2.min_pos.z())),
						Eigen::Vector3f(std::fmax(b1.max_pos.x(), b2.max_pos.x()),
							std::fmax(b1.max_pos.y(), b2.max_pos.y()),
							std::fmax(b1.max_pos.z(), b2.max_pos.z())));
	}
public:
	Eigen::Vector3f min_pos;
	Eigen::Vector3f max_pos;
public:
	BoundingBox()
	{
		//min_pos << std::numeric_limits<float>::lowest(),
		//	std::numeric_limits<float>::lowest(),
		//	std::numeric_limits<float>::lowest();
		//max_pos << std::numeric_limits<float>::max(),
		//	std::numeric_limits<float>::max(),
		//	std::numeric_limits<float>::max();
		min_pos = Eigen::Vector3f::Zero();
		max_pos = Eigen::Vector3f::Zero();
	}

	~BoundingBox() {}

	BoundingBox(const Eigen::Vector3f& tri_p1, const Eigen::Vector3f& tri_p2, const Eigen::Vector3f& tri_p3)
	{
		max_pos << std::fmax(tri_p1.x(), std::fmax(tri_p2.x(), tri_p3.x())),
			std::fmax(tri_p1.y(), std::fmax(tri_p2.y(), tri_p3.y())),
			std::fmax(tri_p1.z(), std::fmax(tri_p2.z(), tri_p3.z()));
		min_pos << std::fmin(tri_p1.x(), std::fmin(tri_p2.x(), tri_p3.x())),
			std::fmin(tri_p1.y(), std::fmin(tri_p2.y(), tri_p3.y())),
			std::fmin(tri_p1.z(), std::fmin(tri_p2.z(), tri_p3.z()));
	}

	BoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max) : min_pos(min), max_pos(max) {}

	Axis LongestAxis() const
	{
		Eigen::Vector3f dis = max_pos - min_pos;
		if (dis.x() > dis.y() && dis.x() > dis.z())
			return Axis::X;
		else if (dis.y() > dis.x() && dis.y() > dis.z())
			return Axis::Y;
		else
			return Axis::Z;
	}

	Eigen::Vector3f Centroid() const { return min_pos * 0.5f + max_pos * 0.5f; };

	bool RayIntersect(const Ray& ray) { return math::AABBTest(ray.ori, ray.inv_dir, min_pos, max_pos); }
};