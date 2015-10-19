
#ifndef IVW_FIELDLINE_H
#define IVW_FIELDLINE_H

//#include <modules/vectorfieldvisualization/vectorfieldvisualizationmoduledefine.h>
#include <bitset>
#include <OpenGL\ext\glew\include\GL\glew.h>
#include <glut\ext\freeglut\include\GL\glut.h>
#include <flow\flowmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/datastructures/geometry/geometrytype.h>


//#include <inviwo/core/properties/transferfunctionproperty.h>
//#include <inviwo/core/properties/minmaxproperty.h>
#include <modules/experimental/datastructures/pointcloud.h>
#include <modules/experimental/ports/pointcloudport.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <modules/base/algorithm/volume/volumeramsubset.h>
#include <inviwo\core\datastructures\buffer\bufferramprecision.h>
//#include <mrivisualization\util\integrator.h>
//#include <mrivisualization/util/typedefsandenums.h>
//#include <mrivisualization\processors\integrallinepostprocessing.h>
//#include <mrivisualization\datastructures\integrallineset.h>


namespace inviwo {

    //class SimpleMesh;
    //class VolumeRAM;
	
	
class IVW_MODULE_FLOW_API FieldLine : public Processor { 
public:
    FieldLine();
    virtual ~FieldLine();

    InviwoProcessorInfo();

    virtual void process();
	
    protected:
	
    VolumeInport inportvol_;
	PointCloudInport inportpoint_;
	MeshOutport outportline_;

    IntProperty numberOfSteps_;
    FloatProperty stepSize_;
	FloatProperty d_;
	IntProperty m_;
    OptionPropertyInt stepDirection_;
	OptionPropertyInt integration_type_;


	TransferFunctionProperty tf_;
	FloatProperty velocityScale_;
	FloatMinMaxProperty minMaxVelocity_;

	struct neighbor{

		 vec3 p1 ;
		 vec3 p2 ;
		 vec3 p3 ;
		 vec3 p4 ;

	};

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
		IndexBufferRAM*);

	std::vector<vec3> GetNeighbour(vec3 n, vec3 seedpoint, int m);

	//float CalculateDistance(vec3& p1, const std::vector<vec3>& p2);

	vec4 velocity2color(const vec3 &veloicty);

	vec2 minMaxVelocityTemp_;



};

} // namespace

#endif // IVW_FIELDLINE_H

