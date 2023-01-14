#include "BVH.h"
#include "Primitive.h"

#include <iostream>

BVHTree::BVHTree(std::vector<std::shared_ptr<Primitive>>& pris)
{
	root = RecursiveBuild(pris);
}

BVHTree::~BVHTree()
{
}

size_t BVHTree::CalculateNodeNum()
{
	size_t nums = 0;
	CalculateNode(nums, root);
	return nums;
}

Intersection BVHTree::Intersect(const Ray& r)
{
	assert(root);
	return IntersectImpl(root, r);
}

Intersection BVHTree::IntersectImpl(std::shared_ptr<BVHNode> node, const Ray& r)
{
	if (!node->box.RayIntersect(r))
		return Intersection();

	if (!node->left && !node->right)
		return node->pris[0]->Intersect(r);

	Intersection hit1, hit2;
	if (node->left)
		hit1 = IntersectImpl(node->left, r);
	if (node->right)
		hit2 = IntersectImpl(node->right, r);

	return hit1.distance < hit2.distance ? hit1 : hit2;
}

std::shared_ptr<BVHTree::BVHNode> BVHTree::RecursiveBuild(std::vector<std::shared_ptr<Primitive>> pris)
{
	std::shared_ptr<BVHTree::BVHNode> node = std::make_shared<BVHNode>();

	BoundingBox box;
	for (auto tri : pris)
		box = Union(tri->box, box);

	if (pris.size() == 1)
	{
		node->box = box;
		node->left = nullptr;
		node->right = nullptr;
		node->pris = std::move(pris);
	}
	else if (pris.size() == 2)
	{
		node->left = RecursiveBuild(std::vector{ pris[0] });
		node->right = RecursiveBuild(std::vector{ pris[1] });

		node->box = Union(node->left->box, node->right->box);
		//node->area = node->left->area + node->right->area;
	}
	else
	{
		switch (box.LongestAxis())
		{
		case Axis::X:
			std::sort(pris.begin(), pris.end(), [](auto f1, auto f2) {
				return f1->box.min_pos.x() < f2->box.min_pos.x();});
			break;
		case Axis::Y:
			std::sort(pris.begin(), pris.end(), [](auto f1, auto f2) {
				return f1->box.min_pos.y() < f2->box.min_pos.y(); });
			break;
		case Axis::Z:
			std::sort(pris.begin(), pris.end(), [](auto f1, auto f2) {
				return f1->box.min_pos.z() < f2->box.min_pos.z(); });
			break;
		default:
			assert(0);
			break;
		}
		auto beginning = pris.begin();
		auto middling = pris.begin() + (pris.size() / 2);
		auto ending = pris.end();

		node->left = RecursiveBuild(std::vector<std::shared_ptr<Primitive>>(beginning, middling));
		node->right = RecursiveBuild(std::vector<std::shared_ptr<Primitive>>(middling, ending));

		node->box = Union(node->left->box, node->right->box);
		//node->area = node->left->area + node->right->area;
	}
	return node;
}

void BVHTree::CalculateNode(size_t& sz, std::shared_ptr<BVHNode> node)
{
	if (node)
	{
		sz++;
		/*std::cout << node->box.min_pos.x() << " " << node->box.min_pos.y() << " " << node->box.min_pos.z() << std::endl;
		std::cout << node->box.max_pos.x() << " " << node->box.max_pos.y() << " " << node->box.max_pos.z() << std::endl;*/
		CalculateNode(sz, node->left);
		CalculateNode(sz, node->right);
	}
}
