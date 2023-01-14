#pragma once
#include <Eigen/Eigen>

struct Ray
{
	Eigen::Vector3f ori;
	Eigen::Vector3f dir;
	Eigen::Vector3f inv_dir;

	Ray() {}

	Ray(const Eigen::Vector3f& ori_, const Eigen::Vector3f& dir_)
		:ori(ori_), dir(dir_)
	{
		inv_dir = Eigen::Vector3f::Ones().cwiseQuotient(dir);
	}
};