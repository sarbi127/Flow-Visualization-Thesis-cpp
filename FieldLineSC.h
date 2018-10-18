
#ifndef IVW_FIELDLINESC_H
#define IVW_FIELDLINESC_H


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

    //class SimpleMesh;
    //class VolumeRAM;
	
	
class IVW_MODULE_FLOW_API FieldLineSC : public Processor { 
public:

	FieldLineSC();
	virtual ~FieldLineSC();

    InviwoProcessorInfo();

    virtual void process();

	FloatProperty dd_;
	
    protected:
	
    VolumeInport inportvolcartesian_;
	VolumeInport inportvolspherical_;

	DataInport<std::vector<vec3>> inportpoint_;
	MeshInport mesh_inport_;

	MeshOutport outportmesh_;
	MeshOutport  outportline_;
	//MeshOutport  outportlinecartesian_;
	
	BoolProperty show_spherical_;
	//BoolProperty show_cartesian_;
	

    IntProperty numberOfSteps_;
    FloatProperty stepSize_;
	FloatProperty d_;
	FloatProperty maxlambda_;
	
	//IntProperty m_;
    OptionPropertyInt stepDirection_;
	OptionPropertyInt integration_type_;


	TransferFunctionProperty tf_;
	FloatProperty velocityScale_;
	FloatMinMaxProperty minMaxVelocity_;

	FloatVec3Property center_;

	struct neighbor{

		 vec3 p1 ;
		 vec3 p2 ;
		 vec3 p3 ;
		 vec3 p4 ;

	};

	float CalculateDistance(vec3& p1, vec3& p2);

	vec4 distance2color(float idx);

	vec3 euler_cartesian(vec3& p, const  VolumeRAM* vector_field, float& stepsize, const int& dir,
		mat3& transformation_matrix, float* velocity);

	vec3 runge_kutta_4_cartesian(vec3& p, const VolumeRAM* vector_field,
		float& stepsize, const int& dir, mat3& transformation_matrix = mat3(1),
		float* = nullptr);

	void step_cartesian(const VolumeRAM* volume,
		BasicMesh* mesh,
		const vec3 &startPos,
		const size_t &steps,
		const int &dir,
	    Line *l);

	vec4 velocity2color(const vec3 &veloicty);

	vec2 minMaxVelocityTemp_;

	float eigenvalues(vec3 p0, vec3 p1, vec3 p2, vec3 p3);

	void CalculateCartesianLines();

	vec3 euler_spherical(vec3& p, const  VolumeRAM* vector_field, float& stepsize, const int& dir,
		mat3& transformation_matrix, float* velocity);

	vec3 runge_kutta_4_spherical(vec3& p, const VolumeRAM* vector_field,
		float& stepsize, const int& dir, mat3& transformation_matrix = mat3(1),
		float* = nullptr);

	void step_spherical(const VolumeRAM* volume,
		BasicMesh* mesh,
		const vec3 &startPos,
		const size_t &steps,
		const int &dir,
		Line *l);

	void CalculateSpericalLines();

	std::vector<vec3> linestrips_;
	std::vector<Line> lines_;

	std::vector<size_t> index_;

	vec3 points;

};




} // namespace

#endif // IVW_FIELDLINESC_H

