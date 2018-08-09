#ifndef IVW_STREAMLINES_H
#define IVW_STREAMLINES_H

#include <modules/vectorfieldvisualization/vectorfieldvisualizationmoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <modules/experimental/ports/pointcloudport.h>
#include <inviwo/core/ports/meshport.h>

namespace inviwo {

    class SimpleMesh;
    class VolumeRAM;
class IVW_MODULE_VECTORFIELDVISUALIZATION_API StreamLines : public Processor { 
public:
    StreamLines();
    virtual ~StreamLines();

    InviwoProcessorInfo();

    virtual void process();
protected:
    VolumeInport volume_;
	PointCloudInport seedPoints_;

    MeshOutport linesStripsMesh_;

    IntProperty numberOfSteps_;
    FloatProperty stepSize_;
    OptionPropertyInt stepDirection_;


    TransferFunctionProperty tf_;
    FloatProperty velocityScale_;
    FloatMinMaxProperty minMaxVelocity_;



    void step(  const VolumeRAM *volume,
                SimpleMesh* mesh , 
                const vec3 &startPos,
                const size_t &steps,
                const int &dir = 1);

    vec4 velocity2color( const vec3 &veloicty );

    vec2 minMaxVelocityTemp_;
};

} // namespace

#endif // IVW_STREAMLINES_H

