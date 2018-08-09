

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
	m_("m_", " m ", 4, 0, 20, 0),
	n_("n_", " n ", 4, 0, 20, 0)

{

	addProperty(center_);
	addProperty(radius_);
    addProperty(n_);
    addProperty(m_);
	addPort(outportpoint_);

	//seedpoints_ = std::make_shared<PointCloud>();
	outportpoint_.setData(&seedpoints_);


}

SeedPoint::~SeedPoint() {}


void SeedPoint::process() {

	seedpoints_.clear();

	//int N= 0;
	//float a = (4 * PI *((radius_.get()) * (radius_.get()))) / N;
	//float d = glm::sqrt(a);

	//int m_ = glm::round(PI / d);

	//float dtheta = PI / m_;
	//float dphi = a / dtheta;

	float xfrac = 360 / m_.get();
	float yfrac = 180 / n_.get();
	float xangle, yangle;

	xangle = yangle = .1f;

	float phi = xangle;
	float theta = yangle;

	for (size_t i = 0; i < m_.get(); i++){
	
		//vec3 dir = glm::rotate(vec3(1, 0, 0), xangle, vec3(0, 1, 0));

		//float theta = (PI * (i + 0.5)) / m_;

		//int n_ = glm::round(2 * PI * sin(theta / dphi));

		phi += xfrac;

		for (size_t j = 0; j < n_.get(); j++){
		
			//dir = glm::rotate(dir, yangle, vec3(1, 0, 0));

			//float phi = (2 * PI * j) / n_;


			float x = radius_.get() * sin(theta) * cos(phi);
			float y = radius_.get() * sin(theta) * sin(phi);
			float z = radius_.get() * cos(theta);

			seedpoints_.push_back(center_.get() + vec3(x, y, z));

			theta += yfrac;

			

			//N += 1;

			//seedpoints_->addPoint(center_.get() + glm::normalize(dir) * radius_.get());

		
		}

	}

}

} // namespace
