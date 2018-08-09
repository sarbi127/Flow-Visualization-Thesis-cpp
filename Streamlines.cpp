#include "streamlines.h"

#include <bitset>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/opengl/image/layergl.h>
#include <inviwo/core/datastructures/image/layerram.h>

#define FWD 1
#define BWD 2
#define BOTH 3

namespace inviwo {

    static vec3 linear(const vec3 &a,const vec3 &b,const float &x){
        return a + (b-a)*x;
    }

    static vec3 bilinear(const vec3 *samples,const vec2 &xy){
        return linear(
                linear(samples[0],samples[1],xy.x),
                linear(samples[2],samples[3],xy.x),
                xy.y);
    }

    static vec3 trilinear(const VolumeRAM* volume,const vec3 &pos){
        uvec3 samplePos = uvec3(pos * vec3(volume->getDimensions()));
        vec3 samples[8];

        vec3 interpolants = (pos * vec3(volume->getDimensions())) - vec3(samplePos);

        samples[0] = (vec3)volume->getValueAsVec3Double(samplePos);
        samples[1] = (vec3)volume->getValueAsVec3Double(samplePos + uvec3(1,0,0));
        samples[2] = (vec3)volume->getValueAsVec3Double(samplePos + uvec3(0,1,0));
        samples[3] = (vec3)volume->getValueAsVec3Double(samplePos + uvec3(1,1,0));

        samples[4] = (vec3)volume->getValueAsVec3Double(samplePos + uvec3(0,0,1));
        samples[5] = (vec3)volume->getValueAsVec3Double(samplePos + uvec3(1,0,1));
        samples[6] = (vec3)volume->getValueAsVec3Double(samplePos + uvec3(0,1,1));
        samples[7] = (vec3)volume->getValueAsVec3Double(samplePos + uvec3(1,1,1));

        vec3 v = linear(
            bilinear(&samples[0],interpolants.xy()),
            bilinear(&samples[4],interpolants.xy()),
            interpolants.z);



        return v;
    }



    ProcessorClassIdentifier(StreamLines, "org.inviwo.StreamLines")
    ProcessorDisplayName(StreamLines,  "StreamLines")
    ProcessorTags(StreamLines, Tags::None);
    ProcessorCategory(StreamLines, "Vector Field Visualization");
    ProcessorCodeState(StreamLines, CODE_STATE_EXPERIMENTAL);


    StreamLines::StreamLines()
        : Processor()
        , volume_("vectorvolume")
        , seedPoints_("seedpoints")
        , linesStripsMesh_("linesStripsMesh_")
        , numberOfSteps_("steps","Number of Steps",100,1,1000)
        , stepSize_("stepSize","StepSize",0.001f,0.0001f,1.0f)
        , stepDirection_("stepDirection","Step Direction")
        , tf_("transferFunction_","Transfer Function")
        , velocityScale_("velocityScale_","Velocity Scale",1,0,2)
        , minMaxVelocity_("minMaxVelocity","Velocity Range",0.0f,1.0f,0.0f,1.0f)
    {
        addPort(volume_);
        addPort(seedPoints_);
        addPort(linesStripsMesh_);

        stepSize_.setIncrement(0.0001f);
        stepDirection_.addOption("fwd","Forward",FWD);
        stepDirection_.addOption("bwd","Backwards",BWD);
        stepDirection_.addOption("bi","Bi Directional",BOTH);

        stepSize_.setCurrentStateAsDefault();
        stepDirection_.setCurrentStateAsDefault();

        minMaxVelocity_.setReadOnly(true);

        addProperty(numberOfSteps_);
        addProperty(stepSize_);
        addProperty(stepDirection_);

        addProperty(tf_);
        addProperty(velocityScale_);
        addProperty(minMaxVelocity_);
        
        
        tf_.get().clearPoints();
        tf_.get().addPoint(vec2(0,1),vec4(0,0,1,1));
        tf_.get().addPoint(vec2(0.5,1),vec4(1,1,0,1));
        tf_.get().addPoint(vec2(1,1),vec4(1,0,0,1));


        tf_.setCurrentStateAsDefault();
    }

    StreamLines::~StreamLines(){

    }



    void StreamLines::process(){
        minMaxVelocityTemp_ = vec2(1000000,0);
        minMaxVelocity_.set(minMaxVelocityTemp_);

        SimpleMesh *mesh = new SimpleMesh(GeometryEnums::LINES,GeometryEnums::NONE);

        mesh->setModelMatrix(volume_.getData()->getModelMatrix());//????????????????????????
        mesh->setWorldMatrix(volume_.getData()->getWorldMatrix());

        linesStripsMesh_.setData(mesh);

        const VolumeRAM* volumeRAM = volume_.getData()->getRepresentation<VolumeRAM>();

        bool fwd = std::bitset<sizeof(int)>(  stepDirection_.get()).test(0);
        bool bwd = std::bitset<sizeof(int)>(  stepDirection_.get()).test(1);

        size_t steps = numberOfSteps_.get();
        if(fwd && bwd){ //go both way
            steps /= 2;
        }

		for (size_t i = 0; i < seedPoints_.getData()->getPoints().size(); i++) {
            if (fwd) {
				step(volumeRAM, mesh, seedPoints_.getData()->getPoints()[i], steps, 1);
            }
            if(bwd){
				step(volumeRAM, mesh, seedPoints_.getData()->getPoints()[i], steps, -1);
            }
        }

        minMaxVelocity_.set(minMaxVelocityTemp_);

    }


    void StreamLines::step( const VolumeRAM *volume,
                            SimpleMesh* mesh,
                            const vec3 &startPos,
                            const size_t &steps,
                            const int &dir)
    {
        vec3 currentPos = startPos;
        vec3 prevPos;
        uvec3 dim = volume->getDimensions();
        for(size_t i = 0;i<steps;i++){
            if(currentPos.x < 0) break;
            if(currentPos.y < 0) break;
            if(currentPos.z < 0) break;
            if(currentPos.x > 1) break;
            if(currentPos.y > 1) break;
            if(currentPos.z > 1) break;

            if(currentPos.x > 1 - 1.0/dim.x) break;
            if(currentPos.y > 1 - 1.0/dim.y) break;
            if(currentPos.z > 1 - 1.0/dim.z) break;

            vec3 velocity =  trilinear(volume,currentPos);

            prevPos = currentPos;
            currentPos += velocity.xyz() * (stepSize_.get() * dir) ;

            vec4 color = velocity2color(velocity.xyz());

            mesh->addVertex(prevPos,prevPos ,color);
            mesh->addVertex(currentPos,currentPos ,color);
        }
    }

    vec4 StreamLines::velocity2color( const vec3 &veloicty ) {
        float d = glm::length(veloicty) * velocityScale_.get();

        minMaxVelocityTemp_.x = std::min(d,minMaxVelocityTemp_.x);
        minMaxVelocityTemp_.y = std::max(d,minMaxVelocityTemp_.y);

        const LayerRAM* tf = tf_.get().getData()->getRepresentation<LayerRAM>();
        if(d>1) d = 1;
        if(d<0) d = 0;
        uvec2 pos(d * (tf->getDimensions().x-1),0);

        return (vec4)tf->getValueAsVec4Double(pos);

    }

} // namespace

