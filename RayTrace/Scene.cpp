#include "Scene.h"

#include "BVH.h"
#include "Primitive.h"


void Scene::AddObject(const std::vector<std::shared_ptr<Primitive>>& pris_)
{
	pris.insert(pris.end(), pris_.begin(), pris_.end());
}

void Scene::AddLight(const std::vector<Light>& lis)
{
	lights.insert(lights.end(), lis.begin(), lis.end());
}

void Scene::BuildBVHTree()
{
	tree = std::make_shared<BVHTree>(pris);

	for (Light& li : lights)
		light_area += li.area;
}

Eigen::Vector3f Scene::CastRay(const Ray& r, int depth)
{
	Intersection inter = Intersect(r);
	if (!inter.happened)
		return Eigen::Vector3f::Zero();
	
	if (inter.emited)
		return inter.mat->emission;

	Eigen::Vector3f wi = -r.dir;
	if (wi.dot(inter.normal) == 0.0f)
		return Eigen::Vector3f::Zero();

	Eigen::Vector3f wo = inter.mat->Sample(wi, inter.normal);
	Ray wo_r(inter.coords, wo);
	Intersection wo_target = Intersect(wo_r);
	if (inter.mat->IsIdealSurface())
	{
		if (wo_target.emited)
		{
			float pdf = 1.0f / light_area;
			return wo_target.mat->emission * (-wo).dot(wo_target.normal) / (wo_target.distance * wo_target.distance) / pdf;
		}
		if (math::GetRandomFloat() < math::RussianRoulette)
			return CastRay(Ray(inter.coords, wo), depth + 1) * inter.mat->PDF(wo, wi, inter.normal) / math::RussianRoulette;
		else
			return Eigen::Vector3f::Zero();
	}

	//float li_pdf = 1.0f;
	//float wo_pdf = inter.mat->PDF(wo, wi, inter.normal);
	//Eigen::Vector3f ws = SampleLight(li_pdf, inter);
	//
	////std::cout << li_pdf << std::endl;

	//float pdf = wo_pdf * 0.5f + li_pdf * 0.5f;
	//Ray wo_fin;
	//if (math::GetRandomFloat() < 0.5)
	//	return CastRay(Ray(inter.coords, ws), depth + 1).cwiseProduct(inter.mat->Eval(ws, wi, inter.normal)) *
	//			inter.mat->PDF(ws, wi, inter.normal) / pdf;
	//else
	//	return CastRay(Ray(inter.coords, wo), depth + 1).cwiseProduct(inter.mat->Eval(wo, wi, inter.normal)) *
	//			inter.mat->PDF(wo, wi, inter.normal) / pdf;

	 //Light direct sample
	Eigen::Vector3f L_dir = Eigen::Vector3f::Zero();
	Eigen::Vector3f L_indir = Eigen::Vector3f::Zero();
	
	float pdf = 1.0f;
	Intersection light_inter = SampleLight(pdf);
	Eigen::Vector3f ws = light_inter.coords - inter.coords;
	Eigen::Vector3f ws_nor = ws.normalized();
	float ws_dis = ws.norm();
	Intersection isLightToTarget = Intersect(Ray(inter.coords, ws_nor));
	if (std::fabs(ws_dis - isLightToTarget.distance) < math::EPSILON) // if epsilon is samll, the image will be dark
		L_dir = light_inter.mat->emission.cwiseProduct(inter.mat->Eval(ws_nor, wi, inter.normal)) *
					ws_nor.dot(inter.normal) * (-ws_nor).dot(light_inter.normal) / (ws_dis * ws_dis) / pdf;
	
	if (math::GetRandomFloat() < math::RussianRoulette)
	{
		if (wo_target.happened && !wo_target.mat->emited)
			L_indir = CastRay(wo_r, depth).cwiseProduct(inter.mat->Eval(wo, wi, inter.normal)) * wo.dot(inter.normal)
						/ inter.mat->PDF(wo, wi, inter.normal) / math::RussianRoulette;
	}
	return L_dir + L_indir;
}

Intersection Scene::Intersect(const Ray& r)
{
	return tree->Intersect(r);
}

Intersection Scene::SampleLight(float& pdf)
{
	return lights[0].Sample(pdf);
//	Intersection light_inter = lights[0].Sample(pdf);
//	Eigen::Vector3f ws = light_inter.coords - inter.coords;
//	Eigen::Vector3f ws_nor = ws.normalized();
//	float ws_dis = ws.norm();
//	Intersection isLightToTarget = Intersect(Ray(inter.coords, ws_nor));
//	bool li_hit = std::fabs(ws_dis - isLightToTarget.distance) < math::EPSILON; // if epsilon is samll, the image will be dark
//	if (!li_hit)
//	{
//		//pdf = 0.0f;
//		float ws_square = ws_dis * ws_dis;
//		float cosine = std::fabs((-ws_nor).dot(light_inter.normal));
//		pdf *= ws_square / cosine;
//	}
//	else
//	{
//		float ws_square = ws_dis * ws_dis;
//		float cosine = std::fabs((-ws_nor).dot(light_inter.normal));
//		pdf *= ws_square / cosine;
//	}
//	return ws_nor;
}
