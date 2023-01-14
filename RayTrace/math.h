#pragma once

#include<iostream>
#include <Eigen/Eigen>
#include <random>

namespace math
{
	const float EPSILON = 1e-3f;

	const float PI = 3.141592653589793f;

	const float RussianRoulette = 0.8f;

	inline float GetRandomFloat()
	{
		thread_local static std::random_device dev;
		thread_local static std::mt19937 rng(dev());
		std::uniform_real_distribution<float> dist(0.0f, 1.0f); // distribution in range [1, 6]

		return dist(rng);
	}

	inline Eigen::Vector3f ToWorld(const Eigen::Vector3f& local, const Eigen::Vector3f& N)
	{
		Eigen::Vector3f W(N);
		Eigen::Vector3f A = Eigen::Vector3f::UnitX();
		if (std::fabs(N.x()) > 0.999f)
			A = Eigen::Vector3f::UnitY();
		Eigen::Vector3f U = W.cross(A);
		Eigen::Vector3f V = W.cross(U);
		return local.x() * U + local.y() * V + local.z() * W;
	}

	inline Eigen::Vector3f UniformSampleHemisphere()
	{
		float x_1 = GetRandomFloat();
		float x_2 = GetRandomFloat();
		float cos_theta = x_1;
		float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
		float phi = 2.0f * math::PI * x_2;
		return { sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta };
	}

	inline Eigen::Vector2f UniformSampleUnitDisk()
	{
		float x_1 = GetRandomFloat();
		float x_2 = GetRandomFloat();
		float r = std::sqrt(x_1);
		float phi = 2 * math::PI * x_2;
		return { r * std::cos(phi), r * std::sin(phi) };
	}

	inline Eigen::Vector3f CosSampleHemisphere()
	{
		float x_1 = GetRandomFloat();
		float x_2 = GetRandomFloat();
		float cos_theta = std::sqrt(1.0f - x_1);
		//float cos_theta = std::pow(1.0f - x_1, 0.5f);.
		float sin_theta = std::sqrt(x_1);
		float phi = 2.0f * math::PI * x_2;
		return { sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta };
	}

	inline Eigen::Vector3f GGXSampleHemisphere(float roughness_square)
	{
		float x_1 = GetRandomFloat();
		float x_2 = GetRandomFloat();
		float cos_theta = std::sqrt((1.0f - x_1) / ((roughness_square - 1.0f) * x_1 + 1.0f));
		float phi = 2.0f * math::PI * x_2;
		float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
		return{ std::cos(phi) * sin_theta, std::sin(phi) * sin_theta, cos_theta };
	}

	inline std::pair<float, Eigen::Vector3f> RayIntersectTriangle0(const Eigen::Vector3f& ray_ori, const Eigen::Vector3f& ray_dir,
		const Eigen::Vector3f& v0, const Eigen::Vector3f& E1, const Eigen::Vector3f& E2)
	{
		Eigen::Vector3f P = ray_dir.cross(E2);
		float det = E1.dot(P);

		Eigen::Vector3f T;
		if (det > 0.0f)
		{
			T = ray_ori - v0;
		}
		else
		{
			T = v0 - ray_ori;
			det = -det;
		}
		// If determinant is near zero, ray lies in plane of triangle
		if (det < EPSILON)
			return { -1.0f, Eigen::Vector3f::Zero() };

		float u = T.dot(P);
		if (u < 0.0f || u > det)
			return { -1.0f, Eigen::Vector3f::Zero() };

		Eigen::Vector3f Q = T.cross(E1);
		float v = ray_dir.dot(Q);
		if (v < 0.0f || u + v > det)
			return { -1.0f, Eigen::Vector3f::Zero() };

		float t = E2.dot(Q);
		float fInvDet = 1.0f / det;
		t *= fInvDet;
		u *= fInvDet;
		v *= fInvDet;

		return { t, Eigen::Vector3f(1.0f - u - v, u, v) };
	}

	inline std::pair<bool, Eigen::Vector3f> RayIntersectTriangle(const Eigen::Vector3f& ray_ori, const Eigen::Vector3f& ray_dir,
		const Eigen::Vector3f& tri_s1, const Eigen::Vector3f& E1, const Eigen::Vector3f& E2)
	{
		Eigen::Vector3f S = ray_ori - tri_s1;
		Eigen::Vector3f S1 = ray_dir.cross(E2);
		Eigen::Vector3f S2 = S.cross(E1);
		Eigen::Vector3f tuv = 1.0f / S1.dot(E1) * Eigen::Vector3f(S2.dot(E2), S1.dot(S), S2.dot(ray_dir));
		float alaph = 1.0f - tuv.y() - tuv.z();
		bool inter = tuv.x() > 0.0f && tuv.y() > 0.0f && tuv.z() > 0.0f && alaph > 0.0f;
		return { inter , Eigen::Vector3f(alaph, tuv.y(), tuv.z()) };
	}

	inline bool RayIntersectPlane(const Eigen::Vector3f& ray_ori, const Eigen::Vector3f& ray_dir,
		const Eigen::Vector3f& plane_N, const Eigen::Vector3f& plane_P)
	{
		float t = (plane_P - ray_ori).dot(plane_N) / ray_dir.dot(plane_N);
		return t >= 0.0f && t < std::numeric_limits<float>::infinity();
	}

	inline bool AABBTest(const Eigen::Vector3f& ray_dir, const Eigen::Vector3f& ray_inv_dir, 
		const Eigen::Vector3f& min_pos, const Eigen::Vector3f& max_pos)
	{
		float t_minX = (min_pos.x() - ray_dir.x()) * ray_inv_dir.x();
		float t_maxX = (max_pos.x() - ray_dir.x()) * ray_inv_dir.x();
		float t_minY = (min_pos.y() - ray_dir.y()) * ray_inv_dir.y();
		float t_maxY = (max_pos.y() - ray_dir.y()) * ray_inv_dir.y();
		float t_minZ = (min_pos.z() - ray_dir.z()) * ray_inv_dir.z();
		float t_maxZ = (max_pos.z() - ray_dir.z()) * ray_inv_dir.z();

		if (ray_inv_dir.x() < 0.0f)
			std::swap(t_minX, t_maxX);
		if (ray_inv_dir.y() < 0.0f)
			std::swap(t_minY, t_maxY);
		if (ray_inv_dir.z() < 0.0f)
			std::swap(t_minZ, t_maxZ);

		float t_enter = std::fmax(std::fmax(t_minX, t_minY), t_minZ);
		float t_exit = std::fmin(std::fmin(t_maxX, t_maxY), t_maxZ);
		return t_enter <= t_exit && t_exit >= 0.0f;
	}

	inline float Deg2Rad(float deg)
	{
		return deg * PI / 180.0f;
	}

	inline float Rad2Deg(float rad)
	{
		return rad * 180.0f / PI;
	}

	inline float SolveQuadratic(float a, float b, float c)
	{
		float discr = b * b - a * c;
		if (discr < 0.0f)
			return -1.0f;

		float x0 = -b - std::sqrtf(discr);
		float x1 = -b + std::sqrtf(discr);
		//if (x0 > x1)
		//	std::swap(x0, x1);

		// very important !!! the intersected point must not itself
		// also need postive
		if (x0 < 0.05f)
			x0 = x1;
		if (x0 < 0.05f)
			return -1.0f;

		return x0;
	}

	inline Eigen::Vector3f Reflect(const Eigen::Vector3f& wi, Eigen::Vector3f N)
	{
		if (wi.dot(N) < 0.0)
			N = -N;
		return -wi + 2.0f * wi.dot(N) * N;
	}

	inline Eigen::Vector3f Refract(const Eigen::Vector3f& wi, Eigen::Vector3f N, float etat)
	{
		float cosi = wi.dot(N);
		float etai = 1.0f;
		if (cosi < 0.0f)
		{
			N = -N;
			cosi = std::fabs(cosi);
			std::swap(etat, etai);
		}
		float eta = etai / etat;
		float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
		if (k < 0.0f)
			return Eigen::Vector3f::Zero();
		return -eta * wi + ((eta * cosi - std::sqrt(k)) * N);
	}

	inline float Fresnel(const Eigen::Vector3f& wi, const Eigen::Vector3f& N, float etat)
	{
		float cosi = wi.dot(N);
		float etai = 1.0f;
		if (cosi < 0.0f)
		{
			std::swap(etat, etai);
			cosi = std::fabs(cosi);
		}
		// Compute sini using Snell's law
		float sint = etai / etat * std::sqrt(1.0f - cosi * cosi);
		// Total internal reflection
		if (sint >= 0.999f) 
			return 1.0f;

		float cost = std::sqrt(1.0f - sint * sint);
		float Rs = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		float Rp = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		return (Rs * Rs + Rp * Rp) / 2.0f;
		// As a consequence of the conservation of energy, transmittance is given by:
		// kt = 1 - kr;
	}

	inline float SchlickFresnel(const Eigen::Vector3f& wi, const Eigen::Vector3f& N, float etat)
	{
		float cosi = wi.dot(N);
		float etai = 1.0f;
		if (cosi < 0.0f)
		{
			std::swap(etat, etai);
			cosi = std::fabs(cosi);
		}
		float idx = etai / etat;
		float sint = idx * std::sqrt(1.0 - cosi * cosi);
		if (sint >= 0.999f)
			return 1.0f;
		// Use Schlick's approximation for reflectance.
		auto r0 = (1 - idx) / (1.0f + idx);
		r0 = r0 * r0;
		return r0 + (1 - r0) * std::pow((1.0f - cosi), 5.0f);
	}

	inline Eigen::Vector3f SchlickFresnel(const Eigen::Vector3f& ks, const Eigen::Vector3f& wm, const Eigen::Vector3f& wi)
	{
		float cos_theta = wi.dot(wm);
		float exponential = std::pow(1.0f - cos_theta, 5.0f);
		return ks + (Eigen::Vector3f::Ones() - ks) * exponential;
	}

	inline float GGXNormalDistribution(const Eigen::Vector3f& wm, const Eigen::Vector3f& N, float roughness_square)
	{
		float cos_theta = wm.dot(N);
		float e1 = (roughness_square - 1.0f) * cos_theta * cos_theta + 1.0f;
		float e2 = math::PI * e1 * e1;
		return roughness_square / e2;
	}

	inline float GGXPDF(const Eigen::Vector3f& wm, const Eigen::Vector3f& wo, const Eigen::Vector3f& N, float roughness_square)
	{
		float cos_theta = wm.dot(N);
		float e0 = roughness_square * cos_theta;
		float e1 = (roughness_square - 1.0f) * cos_theta * cos_theta + 1.0f;
		float e2 = math::PI * e1 * e1;
		return e0 / e2 / (4.0f * wo.dot(wm));
	}

	inline float SmithGGXShadowingMasking(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& N, float roughness_square)
	{
		float dotNL = wi.dot(N);
		float dotNV = wo.dot(N);

		float denomA = dotNV * std::sqrt(roughness_square + (1.0f - roughness_square) * dotNL * dotNL);
		float denomB = dotNL * std::sqrt(roughness_square + (1.0f - roughness_square) * dotNV * dotNV);

		return 2.0f * dotNL * dotNV / (denomA + denomB);
	}
}