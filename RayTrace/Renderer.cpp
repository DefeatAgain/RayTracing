#include "Renderer.h"
#include "math.h"
#include "ThreadPoolExecutor.h"

#include <iostream>
#include <plog/Log.h>

Renderer::Renderer(unsigned width_, unsigned height_, const Eigen::Vector3f& eye_pos_, float fov_):
	width(width_), height(height_), div_width(1.0f / width_), div_height(1.0f / height_),
	framebuffer(width_ * height_),
	eye_pos(eye_pos_), fov(fov_)
{
	scale = std::tan(math::Deg2Rad(fov * 0.5f));
	aspect_ratio = (float)width / height;
	spp = 2048;
	finished.store(0);
}

void Renderer::Render(Scene& scene)
{
	unsigned workers = std::thread::hardware_concurrency();
	int start_progress = 0;
	std::cout << "Render with " << workers << " processes. spp: " << spp << std::endl;
	UpdateProgress(0.0f, start_progress);

	ThreadPoolExecutor executor(workers);

	unsigned m = 0;
	for (unsigned x = 0; x < width; x++)
		for (unsigned y = 0; y < height; y++)
			executor.submit(std::bind(&Renderer::SubRender, this, scene, y, x, m++));

	while (finished.load() != framebuffer.size())
	{
		UpdateProgress((float)finished.load() / framebuffer.size(), start_progress);
		std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 1s
	}
	UpdateProgress(1.0f, start_progress);
}

void Renderer::Pressent()
{
	//PostProcess();

#pragma warning(disable : 4996)
	FILE* fp = fopen("..\\binary.ppm", "wb");
	(void)fprintf(fp, "P6\n%d %d\n255\n", width, height);
	float all = (float)framebuffer.size();
	for (unsigned i = 0; i < framebuffer.size(); ++i) {
		static unsigned char color[3];
		color[0] = (unsigned char)(255 * std::pow(std::clamp(framebuffer[i].x(), 0.0f, 1.0f), 1.0f / 2.2f));
		color[1] = (unsigned char)(255 * std::pow(std::clamp(framebuffer[i].y(), 0.0f, 1.0f), 1.0f / 2.2f));
		color[2] = (unsigned char)(255 * std::pow(std::clamp(framebuffer[i].z(), 0.0f, 1.0f), 1.0f / 2.2f));
		fwrite(color, 1, 3, fp);
	}
	fclose(fp);
}

void Renderer::SubRender(Scene& scene, unsigned x, unsigned y, unsigned m)
{
	//PixelFuture pixel;
	for (int i = 0; i < spp; i++)
	{
		Eigen::Vector3f radiance = scene.CastRay(RayGeneration(x, y), 0);
		if (std::isnan(radiance.x()) || std::isnan(radiance.y()) || std::isnan(radiance.x()))
		{
			std::cout << "radiance: " << radiance << std::endl;
			radiance = Eigen::Vector3f::Zero();
		}
		framebuffer[m] += radiance;
	}
	framebuffer[m] /= static_cast<float>(spp);
	finished++;
}

Ray Renderer::RayGeneration(unsigned x_, unsigned y_)
{
	// first scale image into ndc
	Eigen::Vector2f sample = (math::UniformSampleUnitDisk() * 0.5f).array() + 0.5f;
	float x = (1.0f - 2.0f * (x_ + sample.x()) * div_width) * aspect_ratio * scale;
	float y = (1.0f - 2.0f * (y_ + sample.y()) * div_height) * scale;
	Eigen::Vector3f dir(x, y, 1.0f); // z always  parallel z-axis
	return Ray(eye_pos, dir.normalized());
}

void Renderer::PostProcess()
{
	for (unsigned x = 1; x < width - 1; x++)
	{
		for (unsigned y = 1; y < height - 1; y++)
		{
			unsigned center = y * height + x;
			if (framebuffer[center].isZero())
			{
				unsigned lu = (y + 1) * height + (x - 1);
				unsigned u = (y + 1) * height + x;
				unsigned ru = (y + 1) * height + x + 1;
				unsigned ld = (y - 1) * height + (x - 1);
				unsigned d = (y - 1) * height + x;
				unsigned rd = (y - 1) * height + x + 1;
				unsigned l = center - 1;
				unsigned r = center + 1;

				if (!framebuffer[lu].isZero() && !framebuffer[u].isZero() && !framebuffer[ru].isZero() && !framebuffer[ld].isZero() &&
					!framebuffer[d].isZero() && !framebuffer[rd].isZero() && !framebuffer[l].isZero() && !framebuffer[r].isZero())
				{
					framebuffer[center] += framebuffer[(y + 1) * height + (x - 1)];
					framebuffer[center] += framebuffer[(y + 1) * height + x];
					framebuffer[center] += framebuffer[(y + 1) * height + x + 1];
					framebuffer[center] += framebuffer[(y - 1) * height + (x - 1)];
					framebuffer[center] += framebuffer[(y - 1) * height + x];
					framebuffer[center] += framebuffer[(y - 1) * height + x + 1];
					framebuffer[center] += framebuffer[center - 1];
					framebuffer[center] += framebuffer[center + 1];
					framebuffer[center] /= static_cast<float>(8);
				}
			}
		}
	}
}

void Renderer::UpdateProgress(float progress, int& last_pos)
{
	int barWidth = 80;
	int now = int(progress * 100);
	if (!last_pos || last_pos < now)
		last_pos = now;
	else
		return;

	std::cout << "[";
	int pos = barWidth * progress;
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos) std::cout << "=";
		else if (i == pos) std::cout << ">";
		else std::cout << " ";
	}
	std::cout << "] " << int(progress * 100) << " %\r";
	std::cout.flush();
}
