
#ifndef IVW_SEEDPOINT_H
#define IVW_SEEDPOINT_H

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
<<<<<<< HEAD
//#include <modules/experimental/ports/pointcloudport.h>
=======
#include <modules/experimental/ports/pointcloudport.h>
>>>>>>> origin/master
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <modules/base/algorithm/volume/volumeramsubset.h>
#include <inviwo\core\datastructures\buffer\bufferramprecision.h>
#include <inviwo/core/properties/minmaxproperty.h>


namespace inviwo {
<<<<<<< HEAD
=======

    
	
>>>>>>> origin/master
	
class IVW_MODULE_FLOW_API SeedPoint : public Processor { 
public:
    SeedPoint();
    virtual ~SeedPoint();

    InviwoProcessorInfo();

    virtual void process();
	
    protected:


<<<<<<< HEAD
	DataOutport<std::vector<vec3>> outportpoint_;
=======
	PointCloudOutport outportpoint_;
>>>>>>> origin/master

	FloatVec3Property center_;
	FloatProperty radius_;
	IntProperty n_;
	IntProperty m_;

<<<<<<< HEAD
	std::vector<vec3> seedpoints_;

	//std::shared_ptr<PointCloud> seedpoints_;
=======
	//std::vector<vec3> seedpoints;

	std::shared_ptr<PointCloud> seedpoints_;
>>>>>>> origin/master

};

} // namespace

#endif // IVW_SEEDPOINT_H

