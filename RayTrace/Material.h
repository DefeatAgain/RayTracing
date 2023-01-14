#pragma once
#include <Eigen/Eigen>

#include "math.h"

class Material
{
public:
	enum class MaterialType
	{
		DIFFUSE,
		SPECULAR,
		TRANSPARENT,
		MICROFACET
	};
public:
	static bool HasEmission(const Eigen::Vector3f& irr)
	{
		return irr.norm() > 1.0f;
	}

public:
	Eigen::Vector3f Ka;
	Eigen::Vector3f Kd;
	Eigen::Vector3f Ks;
	Eigen::Vector3f emission;
	bool emited;
	MaterialType mattype;

	float etat;
	float roughness;
	float roughness_square;

	Material(MaterialType mattype_ = MaterialType::DIFFUSE, bool emited_ = false, float etat_ = 0.0f, float roughness_ = 0.0f)
		: emited(emited_), mattype(mattype_), etat(etat_), roughness(roughness_) 
	{
		roughness_square = roughness * roughness;
	}
	~Material() {}

	bool IsIdealSurface()
	{
		return mattype == Material::MaterialType::TRANSPARENT || mattype == Material::MaterialType::SPECULAR;
	}

	
	Eigen::Vector3f Sample(const Eigen::Vector3f& wi, const Eigen::Vector3f& N)
	{
		switch (mattype)
		{
		case MaterialType::DIFFUSE:
		{
			return math::ToWorld(math::CosSampleHemisphere(), N);
			//return math::ToWorld(math::UniformSampleHemisphere(), N);
			break;
		}
		case MaterialType::TRANSPARENT:
		{
			if (math::GetRandomFloat() < math::Fresnel(wi, N, etat))
				return math::Reflect(wi, N);
			else
				return math::Refract(wi, N, etat);
			break;
		}
		case MaterialType::SPECULAR:
		{
			return math::Reflect(wi, N);
			break;
		}
		case MaterialType::MICROFACET:
		{
			Eigen::Vector3f wm = math::ToWorld(math::GGXSampleHemisphere(roughness_square), N);
			return math::Reflect(wi, wm);
			break;
		}
		default:
			break;
		}
		assert(0);
		return Eigen::Vector3f::Zero();
	}

	float PDF(const Eigen::Vector3f& wo, const Eigen::Vector3f& wi, const Eigen::Vector3f& N)
	{
		switch (mattype)
		{
		case MaterialType::DIFFUSE:
		{
			// uniform sample probability 1 / (2 * PI)
			//if (wo.dot(N) > 0.0f)
			//	return 0.5f / math::PI;
			
			// cos sample probability
			if (wo.dot(N) > 0.0f)
				return wo.dot(N) / math::PI;
			return math::EPSILON;
			break;
		}
		case MaterialType::TRANSPARENT:
		case MaterialType::SPECULAR:
		{
			return 1.0f;
			break;
		}
		case MaterialType::MICROFACET:
		{
			if (wo.dot(N) > 0.0f)
			{
				Eigen::Vector3f wm = (wi + wo).normalized();
				float temp = math::GGXPDF(wm, wo, N, roughness_square);
				if (std::isnan(temp))
				{
					std::cout << "wm: " << wm << std::endl;
					std::cout << "GGXPDF: " << temp << std::endl;
				}
				return temp;
			}
			return math::EPSILON;
			break;
		}
		default:
			break;
		}
		assert(0);
		return 1.0f;
	}

	Eigen::Vector3f Eval(const Eigen::Vector3f& wo, const Eigen::Vector3f& wi, const Eigen::Vector3f& N)
	{
		switch (mattype)
		{
		case MaterialType::DIFFUSE:
		{
			if (wo.dot(N) > 0.0f)
				return Kd / math::PI;
			return Eigen::Vector3f::Zero();
			break;
		}
		case MaterialType::SPECULAR:
		case MaterialType::TRANSPARENT:
		{
			return Eigen::Vector3f::Ones();
			break;
		}
		case MaterialType::MICROFACET:
		{
			if (wo.dot(N) > 0.0f)
			{
				Eigen::Vector3f wm = (wi + wo).normalized();
				Eigen::Vector3f F = math::SchlickFresnel(Ks, wm, wi);
				float G = math::SmithGGXShadowingMasking(wi, wo, N, roughness_square);
				float D = math::GGXNormalDistribution(wm, N, roughness_square);
				Eigen::Vector3f temp = F * D * G / (4.0f * wi.dot(N) * wo.dot(N));
				if (std::isnan(temp.x()) || std::isnan(temp.y()) || std::isnan(temp.x()))
				{
					std::cout << "F: " << F << std::endl;
					std::cout << "G: " << G << std::endl;
					std::cout << "D: " << D << std::endl;
					std::cout << "(4.0f * wi.dot(N) * wo.dot(N)): " << (4.0f * wi.dot(N) * wo.dot(N)) << std::endl;
				}
				return  F * D * G / (4.0f * wi.dot(N) * wo.dot(N));
			}
			return Eigen::Vector3f::Zero();
			break;
		}
		default:
			break;
		}
		assert(0);
		return Eigen::Vector3f::Zero();
	}
};