
#ifndef IVW_DISC_H
#define IVW_DISC_H

//#include <modules/vectorfieldvisualization/vectorfieldvisualizationmoduledefine.h>
#include <bitset>
#include <flow\flowmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/datastructures/geometry/geometrytype.h>

#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <modules/base/algorithm/volume/volumeramsubset.h>
#include <inviwo\core\datastructures\buffer\bufferramprecision.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo\core\properties\boolproperty.h>
#include <flow/datastructures/Face.h>


namespace inviwo {

   
	
class IVW_MODULE_FLOW_API Disc : public Processor { 
public:

    Disc();
    virtual ~Disc();

    InviwoProcessorInfo();
    virtual void process();
	
    protected:

	BasicMesh *mesh_;
	MeshOutport outportmesh_;
	DataOutport<std::vector<vec3>> outportpoint_;
	DataOutport<std::vector<Face>> outportface_;
	//DataOutport<BasicMesh> outportmesh_;

	FloatProperty radius_;
	FloatVec3Property center_;
	ButtonProperty subdivide_btn_;
	ButtonProperty subdividebackward_btn_;

	/*struct face {

		size_t p1;
		size_t p2;
		size_t p3;

		face() {}

		face(size_t o, size_t t, size_t f, bool b, bool is_parent, bool is_core) {

			p1 = o;
			p2 = t;
			p3 = f;
			is_on_border_ = b;
			is_parent_ = is_parent;
			is_core_ = is_core;
		}

		bool operator==(face &f){ return (this->p1 == f.p1 && this->p2 == f.p2 && this->p3 == f.p3); }

		bool is_on_border_;
		bool is_parent_;
		bool is_core_;
	};*/

	vec3 getNormal(Face &f) {

		vec3 p1 = vertices_->at(f.p1);
		vec3 p2 = vertices_->at(f.p2);
		vec3 p3 = vertices_->at(f.p3);

		vec3 u = p2 - p1;
		vec3 v = p3 - p1;

		/*vec2 n;

		n[0] = u[1] * v[2] - v[1] * u[2];
		n[1] = u[0] * v[2] - v[0] * u[2];
		n[2] = u[0] * v[1] - v[0] * u[1];

		return n;*/
		return glm::normalize(glm::cross(u, v));

	};

	void subdivide(Face &f, size_t);
	void subdividebackward(Face &f, std::vector<Face> faces);
	void CreateAndSetOutportData();
	void onButtonPress();
	void onButtonPressback();
	bool findMatch(size_t, Face &);
	
	glm::mat4 model_matrix_;
	std::vector<vec3> *vertices_;
	std::vector<Face> *faces_;
	
};

} // namespace

#endif // IVW_DISC_H

