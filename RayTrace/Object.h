#pragma once
#include <vector>
#include <memory>
#include <string>

class Primitive;
class Material;

class MeshTriangle
{
public:
	MeshTriangle(const std::string& filename, std::shared_ptr<Material> mat);
	~MeshTriangle();

	std::vector<std::shared_ptr<Primitive>> pris;
};
