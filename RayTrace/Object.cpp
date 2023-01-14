#include "Object.h"
#include "Triangle.h"
#include "OBJ_Loader.hpp"

MeshTriangle::MeshTriangle(const std::string& file, std::shared_ptr<Material> mat)
{
	objl::Loader loader;
	bool loaded = loader.LoadFile(file);
	assert(loaded);
	assert(loader.LoadedMeshes.size() == 1);

	for (auto& mesh : loader.LoadedMeshes)
	{
		std::vector<Eigen::Vector3f> vertices;
		for (unsigned& indice : mesh.Indices)
		{
			vertices.emplace_back(mesh.Vertices[indice].Position.X, 
								mesh.Vertices[indice].Position.Y, 
								mesh.Vertices[indice].Position.Z);
			/*Eigen::Vector3f tri_1_pos();
			Eigen::Vector3f tri_2_pos(mesh.Vertices[1].Position.X, mesh.Vertices[1].Position.Y, mesh.Vertices[1].Position.Z);
			Eigen::Vector3f tri_3_pos(mesh.Vertices[2].Position.X, mesh.Vertices[2].Position.Y, mesh.Vertices[2].Position/.Z);*/
		}

		assert(vertices.size() % 3 == 0);

		for (size_t i = 0; i < vertices.size(); i += 3)
			pris.emplace_back(std::make_shared<Triangle>(vertices[i], vertices[i + 1], vertices[i + 2], mat));

	}
}

MeshTriangle::~MeshTriangle()
{
}

