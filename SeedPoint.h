
#ifndef IVW_SEEDPOINT_H
#define IVW_SEEDPOINT_H

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


namespace inviwo {

   
	
class IVW_MODULE_FLOW_API SeedPoint : public Processor { 
public:

    SeedPoint();
    virtual ~SeedPoint();

    InviwoProcessorInfo();

    virtual void process();
	
    protected:

	DataOutport<std::vector<vec3>> outportpoint_;

	FloatVec3Property center_;
	FloatProperty radius_;
	IntProperty n_;
	IntProperty m_;

	//std::shared_ptr<PointCloud> seedpoints_;

};

} // namespace

#endif // IVW_SEEDPOINT_H

