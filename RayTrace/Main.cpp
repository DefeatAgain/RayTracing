#include "BVH.h"
#include "Object.h"
#include "Material.h"
#include "Scene.h"
#include "Renderer.h"
#include "Sphere.h"
#include "Triangle.h"

#include <iostream>
#include <chrono>

int main()
{
	std::shared_ptr<Material> red = std::make_shared<Material>();
	std::shared_ptr<Material> green = std::make_shared<Material>();
	std::shared_ptr<Material> brown = std::make_shared<Material>();
	std::shared_ptr<Material> light_m = std::make_shared<Material>();
	std::shared_ptr<Material> glass = std::make_shared<Material>(Material::MaterialType::TRANSPARENT, false, 1.5f);
	std::shared_ptr<Material> specular = std::make_shared<Material>(Material::MaterialType::SPECULAR);
	std::shared_ptr<Material> microfacet = std::make_shared<Material>(Material::MaterialType::MICROFACET, false, 0.0f, 0.1f);

	red->Kd = Eigen::Vector3f(0.63f, 0.065f, 0.05f);
	green->Kd = Eigen::Vector3f(0.14f, 0.45f, 0.091f);
	brown->Kd = Eigen::Vector3f(0.725f, 0.71f, 0.68f);
	microfacet->Ks = Eigen::Vector3f(1.0f, 0.71f, 0.29f);
	//light_m->emission = brown->Kd * 25.0;
	light_m->emission = 8.0f * Eigen::Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
		15.6f * Eigen::Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
		18.4f * Eigen::Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f);
	light_m->emited = true;

	MeshTriangle floor("models\\cornellbox\\floor.obj", brown);
	MeshTriangle shortbox("models\\cornellbox\\shortbox.obj", brown);
	MeshTriangle tallbox("models\\cornellbox\\tallbox.obj", glass);
	MeshTriangle left("models\\cornellbox\\left.obj", red);
	MeshTriangle right("models\\cornellbox\\right.obj", green);
	MeshTriangle light_("models\\cornellbox\\light.obj", light_m);
	Light li(light_.pris, Light::LightType::TRIANGLE);

	std::shared_ptr<Sphere> s1 = std::make_shared<Sphere>(Eigen::Vector3f(184.0f, 246.0f, 173.5f), 80.0f, microfacet); //245 82
	std::cout << "Load Object completely" << std::endl;

	//tallbox.pris[8]->material = specular;
	//tallbox.pris[9]->material = specular;
	/*shortbox.pris[0]->material = specular;
	shortbox.pris[1]->material = specular;*/
	//float delt = -80.0f;
	//std::dynamic_pointer_cast<Triangle>(shortbox.pris[0])->v0.y() += delt;
	//std::dynamic_pointer_cast<Triangle>(shortbox.pris[0])->v1.y() += delt;
	//std::dynamic_pointer_cast<Triangle>(shortbox.pris[0])->v2.y() += delt;
	//std::dynamic_pointer_cast<Triangle>(shortbox.pris[0])->Reset();
	//std::dynamic_pointer_cast<Triangle>(shortbox.pris[1])->v0.y() += delt;
	//std::dynamic_pointer_cast<Triangle>(shortbox.pris[1])->v1.y() += delt;
	//std::dynamic_pointer_cast<Triangle>(shortbox.pris[1])->v2.y() += delt;
	//std::dynamic_pointer_cast<Triangle>(shortbox.pris[1])->Reset();


	Scene scene;
	//scene.AddObject(shortbox.pris[0]);
	//scene.AddObject(shortbox.pris[1]);
	scene.AddObject(floor.pris);
	scene.AddObject(shortbox.pris);
	scene.AddObject(tallbox.pris);
	scene.AddObject(left.pris);
	scene.AddObject(right.pris);
	scene.AddObject(light_.pris);
	scene.AddObject(s1);
	scene.AddLight(std::vector<Light>{ li });
	scene.BuildBVHTree();
	std::cout << "BVH build completely" << std::endl;
	std::cout << "BVH node: " << scene.BVHNodeNum() << std::endl;

	Eigen::Vector3f eye_pos(278, 274, -800);
	Renderer r(784, 784, eye_pos, 40.0f);
	auto start_d1 = std::chrono::high_resolution_clock::now();
	r.Render(scene);
	auto end_d2 = std::chrono::high_resolution_clock::now();
	r.Pressent();
	//std::cout << o.bvh_tree->CalculateNodeNum() << std::endl;
	auto spend_time = std::chrono::duration_cast<std::chrono::seconds>(end_d2 - start_d1).count();
	std::cout << std::endl;
	std::cout << "time spend: " << spend_time << std::endl;
	std::cout << "            " << spend_time / 3600 << " hours." << std::endl;
	std::cout << "            " << spend_time / 60 % 60 << " min." << std::endl;
	std::cout << "            " << spend_time % 60 << " sec." << std::endl;
	//system("pause");
	return 0;
}