
#ifndef IVW_FIELDLINESPHERICALCO_H
#define IVW_FIELDLINESPHERICALCO_H


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
#include <flow/datastructures/Face.h>




namespace inviwo {
		
class IVW_MODULE_FLOW_API FieldLineSphericalCo : public Processor { 
public:

    FieldLineSphericalCo();
    virtual ~FieldLineSphericalCo();

    InviwoProcessorInfo();

    virtual void process();

	void updateColorLine();

    protected:

    VolumeInport inportvol_;
	MeshInport mesh_inport_;
	DataInport<std::vector<vec3>> inportpoint_;	
	DataInport<std::vector<Face>> inportface_;

	MeshOutport outportmesh_;
	MeshOutport  outportline_;
	
    IntProperty numberOfSteps_;
    FloatProperty stepSize_;
	FloatProperty d_Neighbor;
	FloatProperty d_NextPoint;
	FloatProperty d_trim_;
	FloatProperty Power_;

	FloatProperty maxlambda_;
	FloatProperty velocityScale_;
    OptionPropertyInt stepDirection_;
	OptionPropertyInt integration_type_;
	OptionPropertyInt colorvertex_;
	OptionPropertyInt colorline_;
	TransferFunctionProperty tf_;	
	//FloatMinMaxProperty minMaxVelocity_;
	FloatVec3Property center_;
	BoolProperty spherical_pro_;
	//IntProperty m_;

	/*struct neighbor{

		 vec3 p1 ;
		 vec3 p2 ;
		 vec3 p3 ;
		 vec3 p4 ;

	};*/

	float CalculateDistance(vec3& p1, vec3& p2);
	vec4 velocity2color(const vec3 &veloicty);
	vec4 idx2color(float idx);
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
	float radianToDegree(float rad);
	vec2 minMaxVelocityTemp_;
	float eigenvalues(vec3 p0, vec3 p1, vec3 p2, vec3 p3);
	void Calculate4Neighbor(std::vector<vec3> points, const VolumeRAM *volume, BasicMesh *mesh);
	void FTLE_Low_Resolution(BasicMesh *mesh_sphere);
	void FTLE_High_Resolution(std::vector<Face> faces, BasicMesh *mesh_sphere);
	void Dot_Posnor_Vec(BasicMesh *mesh_sphere);
	void CalculateVertexColor(std::vector<Face> faces, std::vector<vec3> points, const VolumeRAM *volume, BasicMesh *mesh);
	void ColorLines(BasicMesh *mesh);
	

	std::vector<vec3> linestrips_;
	std::vector<Line> lines_;
	std::vector<size_t> index_;
	std::vector<vec4> colorFTLE_;
	std::vector<vec4> colorDot_;
	std::vector<float> vel_;
	vec3 points;

};

} // namespace

#endif // IVW_FIELDLINESPHERICALCO_H

