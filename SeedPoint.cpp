

#include "SeedPoint.h"
#include <inviwo/core/common/inviwo.h>
#include <bitset>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/opengl/image/layergl.h>
#include <inviwo/core/datastructures/image/layerram.h>


//#define PI 3.14


namespace inviwo {

ProcessorClassIdentifier(SeedPoint, "org.inviwo.SeedPoint")
ProcessorDisplayName(SeedPoint, "SeedPoint")
ProcessorTags(SeedPoint, Tags::None);
ProcessorCategory(SeedPoint, "SeedPoint Visualization");
ProcessorCodeState(SeedPoint, CodeState::Experimental);

SeedPoint::SeedPoint()
    : Processor(), 

	outportpoint_("seedpoint_"),

	center_("sphereCenter", "Center", vec3(0.5f, 0.5f, 0.5f), vec3(0, 0, 0), vec3(1, 1, 1)),
	radius_("sphereRadius","Radius"),
	n_("m_", " m ", 4, 0, 20, 0),
	m_("n_", " n ", 4, 0, 20, 0)

{

	addPort(outportpoint_);

	addProperty(center_);
	addProperty(radius_);
    addProperty(n_);
	addProperty(m_);

}

SeedPoint::~SeedPoint() {}

void SeedPoint::process() {

	std::vector<vec3> *seedpoints = new std::vector<vec3>();

	float xfrac = M_PI / (static_cast<float>(m_.get()) + 1);
	float yfrac = (2 * M_PI) / static_cast<float>(n_.get());

	float phi = 0.f;
	float theta = yfrac / 2;

	for (size_t i = 0; i < m_.get(); i++){
	
		phi += xfrac;

		for (size_t j = 0; j < n_.get(); j++){

			//Spherical coordinates to Cartesian coordinates
			float x = radius_.get() * sin(theta)* cos(phi);
			float y = radius_.get() * sin(theta) * sin(phi);
			float z = radius_.get() * cos(theta);

			seedpoints->push_back(center_.get() + vec3(x, y, z));

			theta += yfrac;


		}

	}

	outportpoint_.setData(seedpoints);
}

} // namespace
