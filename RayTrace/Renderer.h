#pragma once
#include "Ray.h"
#include "Scene.h"
#include "PixelFuture.h"

#include <atomic>

class Renderer
{
public:
	Renderer(unsigned width, unsigned height, const Eigen::Vector3f& eye_pos, float fov);
	~Renderer() {}

	void Render(Scene& scene);

	void Pressent();

private:
	void SubRender(Scene& scene, unsigned x, unsigned y, unsigned m);
	Ray RayGeneration(unsigned x, unsigned y);

	void PostProcess();
private:

	unsigned width;
	unsigned height;

	float div_width;
	float div_height;

	Eigen::Vector3f eye_pos;

	float fov; // for Vertical vision 
	float scale; //for Vertical vision scale;
	float aspect_ratio; // for Horizon vision scale

	std::vector<Eigen::Vector3f> framebuffer;

	unsigned spp;
	std::atomic<unsigned> finished;
private:
	void UpdateProgress(float progress, int& last_pos);
};

