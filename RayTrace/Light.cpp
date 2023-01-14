#include "Light.h"
#include "Primitive.h"

Light::Light(const std::vector<std::shared_ptr<Primitive>>& pris_, LightType lt_):
	area(0.0f), lt(lt_)
{
	for (auto& pri : pris_)
	{
		pris.emplace_back(pri);
		area += pri->area;
	}
}

Intersection Light::Sample(float& pdf)
{
	pdf = 1.0f / area;// pdf = 1 / A
	switch (lt)
	{
	case LightType::TRIANGLE:
	{
		unsigned index = static_cast<unsigned>(math::GetRandomFloat() * pris.size()) % pris.size();
		return pris[index]->Sample();
		break;
	}
	default:
		break;
	}
	return Intersection();
}
