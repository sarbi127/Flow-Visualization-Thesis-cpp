
#ifndef IVW_FIELDLINECARTESIANCO_H
#define IVW_FIELDLINECARTESIANCO_H


#include <bitset>
#include <modules/vectorfieldvisualization/vectorfieldvisualizationmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <flow\flowmoduledefine.h>

#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>

#include <inviwo/core/properties/templateproperty.h>

#include <inviwo/core/datastructures/geometry/geometrytype.h>
#include <inviwo\core\datastructures\image\layer.h>
#include <inviwo\core\datastructures\image\layerram.h>

#include <inviwo/core/processors/processor.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <modules/base/algorithm/volume/volumeramsubset.h>
#include <inviwo\core\datastructures\buffer\bufferramprecision.h>
#include <inviwo\core\datastructures\geometry\basicmesh.h>
#include <flow/datastructures/line.h>
#include <mrivisualization/util/utilityfunctions.h>
#include <modules\eigenutils\eigenutils.h>
#include <Eigen/Eigenvalues> 




namespace inviwo {
		
class IVW_MODULE_FLOW_API FieldLineCartesianCo : public Processor { 
public:

    FieldLineCartesianCo();
    virtual ~FieldLineCartesianCo();

    InviwoProcessorInfo();

    virtual void process();

	FloatProperty dd_;
	
    protected:
	
    VolumeInport inportvol_;
	DataInport<std::vector<vec3>> inportpoint_;
	MeshInport mesh_inport_;

	MeshOutport outportmesh_;
	MeshOutport  outportline_;
	
    IntProperty numberOfSteps_;

    FloatProperty stepSize_;
	FloatProperty d_;
	FloatProperty maxlambda_;
	FloatProperty velocityScale_;
	
	//IntProperty m_;
    OptionPropertyInt stepDirection_;
	OptionPropertyInt integration_type_;
	OptionPropertyInt color_;


	TransferFunctionProperty tf_;
	
	FloatMinMaxProperty minMaxVelocity_;

	FloatVec3Property center_;

	//BoolProperty spherical_pro_;

	struct neighbor{

		 vec3 p1 ;
		 vec3 p2 ;
		 vec3 p3 ;
		 vec3 p4 ;

	};

	struct face{

		vec3 p1;
		vec3 p2;
		vec3 p3;

		face() {}

		face(vec3 o, vec3 t, vec3 f) {

			p1 == o;
			p2 == t;
			p3 == f;
			

		}


	};

	struct line{

		vec3 p1;
		vec3 p2;

		line() {}

		line(vec3 o, vec3 t) {
			p1 == o;
			p2 == t;

		}
	};


	std::vector<face> face_;
	std::vector<line> line_;
	std::vector<vec3> point_;

	float CalculateDistance(vec3& p1, vec3& p2);

	vec4 distance2color(float idx);

	vec3 euler(vec3& p, const  VolumeRAM* vector_field, float& stepsize, const int& dir,
		mat3& transformation_matrix, float* velocity);

	vec3 runge_kutta_4(vec3& p, const VolumeRAM* vector_field,
		float& stepsize, const int& dir, mat3& transformation_matrix = mat3(1),
		float* = nullptr);

	void step(const VolumeRAM* volume,
		BasicMesh* mesh,
		const vec3 &startPos,
		const size_t &steps,
		const int &dir,
	    Line *l);

	vec4 velocity2color(const vec3 &veloicty);

	vec2 minMaxVelocityTemp_;

	float radianToDegree(float rad);

	float eigenvalues(vec3 p0, vec3 p1, vec3 p2, vec3 p3);

	void Calculate4Neighbor(std::vector<vec3> points, const VolumeRAM *volume, BasicMesh *mesh);

	void CalculateVertexColor(const VolumeRAM *volume, BasicMesh *mesh, std::vector<vec3> points);

	void AddColorLinesToMesh(BasicMesh *mesh);

	std::vector<vec3> linestrips_;
	std::vector<Line> lines_;

	std::vector<size_t> index_;

	vec3 points;

};



} // namespace

#endif // IVW_FIELDLINECARTESIANCO_H

