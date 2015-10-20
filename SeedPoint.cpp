

#include "SeedPoint.h"

#include <bitset>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/opengl/image/layergl.h>
#include <inviwo/core/datastructures/image/layerram.h>


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
	n_("m_", " n ", 20, 0, 1, 0),
	m_("n_", " m ", 20, 0, 1, 0)

{
	addPort(outportpoint_);

	addProperty(radius_);
    addProperty(n_);
	addProperty(m_);

	seedpoints_ = std::make_shared<PointCloud>();
	outportpoint_.setData(seedpoints_);


}

SeedPoint::~SeedPoint() {}



void SeedPoint::process() {

	float xfrac = 360 / m_;
	float yfrac = 360 / n_;

	float xangle, yangle = 0;

	for (size_t i = 0; i < m_.get(); i++){
	
		vec3 dir = glm::rotate(vec3(1, 0, 0), xangle, vec3(0, 1, 0));

		xangle += xfrac;

		for (size_t j = 0; j < n_.get(); j++){
		
			dir = glm::rotate(dir, yangle, vec3(1, 0, 0));

			yangle += yfrac;

			seedpoints_->addPoint(center_.get() + glm::normalize(dir) * radius_.get());
		
		}

	}

}

} // namespace
