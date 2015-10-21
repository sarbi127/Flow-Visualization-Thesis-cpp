

#include "SeedPoint.h"

#include <bitset>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/opengl/image/layergl.h>
#include <inviwo/core/datastructures/image/layerram.h>

#define PI 3.14

namespace inviwo {

ProcessorClassIdentifier(SeedPoint, "org.inviwo.SeedPoint")
ProcessorDisplayName(SeedPoint, "SeedPoint")
ProcessorTags(SeedPoint, Tags::None);
ProcessorCategory(SeedPoint, "SeedPoint Visualization");
ProcessorCodeState(SeedPoint, CODE_STATE_EXPERIMENTAL);

SeedPoint::SeedPoint()
    : Processor(), 

	outportpoint_("seedpoint_"),

	center_("sphereCenter", "Center", vec3(0.5f, 0.5f, 0.5f), vec3(0, 0, 0), vec3(1, 1, 1)),
	radius_("sphereRadius","Radius"),
	n_("m_", " m ", 4, 0, 1, 0),
	m_("n_", " n ", 4, 0, 1, 0)

{
	addPort(outportpoint_);

	addProperty(center_);
	addProperty(radius_);
    addProperty(n_);
	addProperty(m_);

	outportpoint_.setData(&seedpoints_);
}

SeedPoint::~SeedPoint() {}

void SeedPoint::process() {

	seedpoints_.clear();

	//float xfrac = 360.f / static_cast<float>(m_.get());
	//float yfrac = 180.f / static_cast<float>(n_.get());

	//float xangle, yangle;
	//xangle = yangle = .1f;

	//float phi = xangle;
	//float theta = yangle;

	int N = 0;

	float a = (4 * PI *((radius_.get()) * (radius_.get()))) / N;

	float d = glm::sqrt(a);

	m_ = glm::round(PI / d);

	float dtheta = PI / m_.get();
	float dphi = a / dtheta;
	
	for (size_t i = 0; i < m_.get()-1; i++){
	
		//vec3 xdir = glm::rotate(vec3(1, 0, 0), xangle, vec3(0, 1, 0));

		//xangle += xfrac;
		
		float theta = (PI * (i + 0.5)) / m_.get();

		n_ = glm::round(2 * PI * sin(theta / dphi));

		//phi += xfrac;

		for (size_t j = 0; j < n_.get()-1; j++){

			float phi = (2 * PI * j) / n_.get();

			float x = radius_.get() * sin(theta)* cos(phi);
			float y = radius_.get() * sin(theta) * sin(phi);
			float z = radius_.get() * cos(theta);

			
			seedpoints_.push_back(vec3(x, y, z));

			N += 1;


			//theta += yfrac;

			//vec3 ydir = glm::rotate(vec3(0, 1, 0), yangle, vec3(1, 0, 0));

			//yangle += yfrac;

			//vec3 dir = glm::normalize(ydir + xdir);

			//seedpoints_.push_back(center_.get() + dir * radius_.get());	

		}
	}
}

} // namespace
