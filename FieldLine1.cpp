

#include "FieldLine.h"

#include <bitset>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/opengl/image/layergl.h>
#include <inviwo/core/datastructures/image/layerram.h>
//#include <modules/experimental/ports/pointcloudport.h>

#define FWD 1
#define BWD 2
#define BOTH 3


#define EULER 1
#define RUNGE_KUTTA_4 2

namespace inviwo {

	/*static float f(double x, double y){

			return -x / y;

			}*/

	/*static vec3 Euler(const vec3 &pos, const size_t step){

			   float slop = f(pos.x, pos.y);

			   pos.x = pos.x + step;
			   pos.y = pos.y + step * slop;

			   return pos.y;
			   }*/

	// Interpolation
	static vec3 linear(const vec3 &a, const vec3 &b, const float &x) {
		return a + (b - a) * x;
	}

	static vec3 bilinear(const vec3 *samples, const vec2 &xy) {
		return linear(linear(samples[0], samples[1], xy.x),
			linear(samples[2], samples[3], xy.x), xy.y);
	}

	static vec3 trilinear(const VolumeRAM *volume, vec3 &pos) {
		if (pos.x > 1)
			pos.x = 1;
		if (pos.x < 0)
			pos.x = 0;
		if (pos.y > 1)
			pos.y = 1;
		if (pos.y < 0)
			pos.y = 0;
		if (pos.z > 1)
			pos.z = 1;
		if (pos.z < 0)
			pos.z = 0;

		vec3 dim = vec3(volume->getDimensions() - size3_t(1));
		size3_t samplePos = size3_t(pos * dim);
		vec3 samples[8];

		vec3 interpolants = (pos * dim) - vec3(samplePos);

		bool is_right_border = samplePos.x == volume->getDimensions().x - 1;
		bool is_top_border = samplePos.y == volume->getDimensions().y - 1;
		bool is_back_border = samplePos.z == volume->getDimensions().z - 1;

		samples[0] = (vec3)volume->getValueAsVec3Double(samplePos);
		samples[1] = (vec3)volume->getValueAsVec3Double(samplePos + size3_t(!is_right_border, 0, 0));
		samples[2] = (vec3)volume->getValueAsVec3Double(samplePos + size3_t(0, !is_top_border, 0));
		samples[3] = (vec3)volume->getValueAsVec3Double(samplePos + size3_t(!is_right_border, !is_top_border, 0));

		samples[4] = (vec3)volume->getValueAsVec3Double(samplePos + size3_t(0, 0, !is_back_border));
		samples[5] = (vec3)volume->getValueAsVec3Double(samplePos + size3_t(!is_right_border, 0, !is_back_border));
		samples[6] = (vec3)volume->getValueAsVec3Double(samplePos + size3_t(0, !is_top_border, !is_back_border));
		samples[7] = (vec3)volume->getValueAsVec3Double(samplePos + size3_t(!is_right_border, !is_top_border, !is_back_border));

		vec3 v = linear(bilinear(&samples[0], interpolants.xy()),
			bilinear(&samples[4], interpolants.xy()), interpolants.z);

		return v;
	}

ProcessorClassIdentifier(FieldLine, "org.inviwo.FieldLine")
ProcessorDisplayName(FieldLine, "FieldLine")
ProcessorTags(FieldLine, Tags::None);
ProcessorCategory(FieldLine, "Vector Field Visualization");
ProcessorCodeState(FieldLine, CODE_STATE_EXPERIMENTAL);

FieldLine::FieldLine()
    : Processor(), inportvol_("vectorvolume"), inportpoint_("PointCloud"),
      outportline_("linesStripsMesh_")

      ,
	  d_("d_", "distance", 0.01, 0.001, 0.1, 0.001),
	  m_("m_", "neighbor point", 20 , 0, 1, 0),
      numberOfSteps_("steps", "Number of Steps", 100, 1, 1000),
      stepSize_("stepSize", "StepSize", 0.001f, 0.0001f, 1.0f),
      stepDirection_("stepDirection", "Step Direction"),
	  integration_type_("integration_type", "Integration Type")

      ,
      tf_("transferFunction_", "Transfer Function"),
      velocityScale_("velocityScale_", "Velocity Scale", 1, 0, 2),
      minMaxVelocity_("minMaxVelocity", "Velocity Range", 0.0f, 1.0f, 0.0f,
                      1.0f)

{
  addPort(inportvol_);
  addPort(inportpoint_);
  addPort(outportline_);

  stepSize_.setIncrement(0.0001f);

  stepDirection_.addOption("fwd", "Forward", FWD);
  stepDirection_.addOption("bwd", "Backwards", BWD);
  stepDirection_.addOption("bi", "Bi Directional", BOTH);


  integration_type_.addOption("euler", "Euler", EULER);
  integration_type_.addOption("rk4", " Runge-Kutta 4th Order", RUNGE_KUTTA_4);

  
  addProperty(d_);
  addProperty(m_);
  stepSize_.setCurrentStateAsDefault();
  stepDirection_.setCurrentStateAsDefault();

  minMaxVelocity_.setReadOnly(true);

  addProperty(numberOfSteps_);
  addProperty(stepSize_);
  addProperty(stepDirection_);
  addProperty(integration_type_);

  addProperty(tf_);
  addProperty(velocityScale_);
  addProperty(minMaxVelocity_);

  tf_.get().clearPoints();
  tf_.get().addPoint(vec2(0, 1), vec4(0, 0, 1, 1));
  tf_.get().addPoint(vec2(0.5, 1), vec4(1, 1, 0, 1));
  tf_.get().addPoint(vec2(1, 1), vec4(1, 0, 0, 1));

  tf_.setCurrentStateAsDefault();
}

FieldLine::~FieldLine() {}

vec3 FieldLine::euler(vec3& p, const VolumeRAM* vector_field, float& stepsize, const int& dir,
	mat3& transformation_matrix, float* velocity) {
	vec3 K1 = trilinear(vector_field, p);
	vec3 vel = transformation_matrix * K1;
	if (velocity != nullptr) {
		velocity[0] = vel.x;
		velocity[1] = vel.y;
		velocity[2] = vel.z;
	}
	return p + vel * stepsize * static_cast<float>(dir);
}

vec3 FieldLine::runge_kutta_4(vec3& p, const VolumeRAM*  vector_field, float& stepsize,
	const int& dir, mat3& transformation_matrix, float* velocity) {
	float h = stepsize / 2.f;
	vec3 K1 = trilinear(vector_field, p);
	//t = (t + h) > 1.f ? (t + h) - 1.f : t + h;
	vec3 new_pos = p + h * (transformation_matrix * K1);
	if (new_pos.x < .0) { new_pos.x = .0; }
	if (new_pos.y < .0) { new_pos.y = .0; }
	if (new_pos.z < .0) { new_pos.z = .0; }
	if (new_pos.x > 1.) { new_pos.x = 1.; }
	if (new_pos.y > 1.) { new_pos.y = 1.; }
	if (new_pos.z > 1.) { new_pos.z = 1.; }
	vec3 K2 = trilinear(vector_field, new_pos);
	new_pos = p + h * (transformation_matrix * K2);
	if (new_pos.x < .0) { new_pos.x = .0; }
	if (new_pos.y < .0) { new_pos.y = .0; }
	if (new_pos.z < .0) { new_pos.z = .0; }
	if (new_pos.x > 1.) { new_pos.x = 1.; }
	if (new_pos.y > 1.) { new_pos.y = 1.; }
	if (new_pos.z > 1.) { new_pos.z = 1.; }
	vec3 K3 = trilinear(vector_field, new_pos);
	//t = (t + h) > 1.f ? (t + h) - 1.f : t + h;
	new_pos = p + stepsize * (transformation_matrix * K3);
	if (new_pos.x < .0) { new_pos.x = .0; }
	if (new_pos.y < .0) { new_pos.y = .0; }
	if (new_pos.z < .0) { new_pos.z = .0; }
	if (new_pos.x > 1.) { new_pos.x = 1.; }
	if (new_pos.y > 1.) { new_pos.y = 1.; }
	if (new_pos.z > 1.) { new_pos.z = 1.; }
	vec3 K4 = trilinear(vector_field, new_pos);
	vec3 vel = (transformation_matrix * K1) * (1.f / 6.f) + (transformation_matrix * K2) * (1.f / 3.f) +
		(transformation_matrix * K3) * (1.f / 3.f) + (transformation_matrix * K4) * (1.f / 6.f);
	if (velocity != nullptr) {
		velocity[0] = vel.x;
		velocity[1] = vel.y;
		velocity[2] = vel.z;
	}
	return p + vel * stepsize * static_cast<float>(dir);
}

void FieldLine::process() {

  minMaxVelocityTemp_ = vec2(1000000, 0);
  minMaxVelocity_.set(minMaxVelocityTemp_);

  BasicMesh *mesh = new BasicMesh();

  mesh->setModelMatrix(inportvol_.getData()->getModelMatrix());
  mesh->setWorldMatrix(inportvol_.getData()->getWorldMatrix());

  auto inputvolume = inportvol_.getData();

  auto dims = inputvolume->getDimensions();

  auto seedpoints = inportpoint_.getData().get();
//  auto seedpoints = seedpoints_pc->getPoints();

  const VolumeRAM *volume =
      inportvol_.getData()->getRepresentation<VolumeRAM>();

  mat4 index_to_texture =
      inputvolume->getCoordinateTransformer().getIndexToTextureMatrix();
  mat4 model_to_texture =
      inputvolume->getCoordinateTransformer().getModelToTextureMatrix();

  // Spherical coordinate to Cartesian coordinate
  /*
  for (size_t x = 0; x < dims.x; x++)
  {
          for (size_t y = 0; y < dims.y; y++)
          {
                  for (size_t z = 0; z < dims.z; z++)
                  {

                          auto spherical_coords =
  volume->getValueAsVec3Double(size3_t(x, y, z));

                          float z_c = spherical_coords.x *
  cos(spherical_coords.z);
                          float y_c = spherical_coords.x *
  sin(spherical_coords.z) * sin(spherical_coords.y);
                          float x_c = spherical_coords.x *
  sin(spherical_coords.z) * cos(spherical_coords.y);

                          vec3 new_pos = (model_to_texture * vec4(x_c, y_c, z_c,
  1)).xyz;
                          //vec3 new_pos = vec3(x_c, y_c, z_c);
                          dvec3 val = dvec3(new_pos.x, new_pos.y, new_pos.z);

                          cart_vol->setValueFromVec3Double(size3_t(x, y, z),
  val);// = tgt::svec3(x_c, y_c, z_c); // setSpacing or setVoxelFloat?

                          //vox->x = x_c;
                          //vox->y = y_c;
                          //vox->z = z_c;


                  }
          }
  }
  */

  // Seed Points
  /*
  for (size_t x = 0; x < dims.x; x += 64)
  {
          for (size_t y = 0; y < dims.y; y += 64)
          {
                  for (size_t z = 0; z < dims.z; z += 64)
                  {
                          seedpoints.push_back((index_to_texture * vec4(x, y, z, 1)).xyz);
                  }
          }
  }
  */

  bool fwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(0); // False
  bool bwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(1); // True

  size_t steps = numberOfSteps_.get();
  if (fwd && bwd) { // go both way
    steps /= 2;
  }


  for (auto it = seedpoints->begin(); it != seedpoints->end(); ++it) {

		  IndexBufferRAM *index =
			  mesh->addIndexBuffer(DrawType::LINES, ConnectivityType::NONE); // Buffer

		  vec3 pos = *it;
		  vec3 N = trilinear(volume, pos);


		/* std::vector<vec3> Npoints = GetNeighbour(N, *it, m_.get());
		  Npoints.push_back(*it);
		  for (auto p : Npoints){

			  if (fwd) {

				  step(volume, mesh, p, steps, 1, index);

			  }
			  if (bwd) {

				  step(volume, mesh, p, steps, -1, index);

			  }
		  } */
		  

	      //size3_t currentPosIndexSpace = size3_t(*it * vec3(inportvol_.getData()->getDimensions()));//textur coor to Index coor(0,1)

	     //p.p1 = (currentPosIndexSpace + vec3(1, 0, 0)) / vec3(inportvol_.getData()->getDimensions());//divide it to back to textur coor

		  //////////////////////////// 4 neighbor point ///////////////////////////////
		  
		/* neighbor p;
		 
		 vec3 w = glm::normalize(glm::cross(N, vec3(0, 0, 1)));

		 p.p1 = *it + d_.get() * w;

		  if (fwd) {

			  step(volume, mesh, p.p1, steps, 1, index);

		  }
		  if (bwd) {

			  step(volume, mesh, p.p1, steps, -1, index);

		  }

		  p.p2 = *it - d_.get() * w;

		  //LogInfo("p2: " << p.p2);

		  if (fwd) {

			  step(volume, mesh, p.p2, steps, 1, index);

		  }
		  if (bwd) {

			  step(volume, mesh, p.p2, steps, -1, index);

		  }

		  vec3 a = glm::normalize(glm::cross(N, w));

		  p.p3 = *it + d_.get() * a;

		  if (fwd) {

			  step(volume, mesh, p.p3, steps, 1, index);

		  }
		  if (bwd) {

			  step(volume, mesh, p.p3, steps, -1, index);

		  }

		  p.p4 = *it - d_.get() * a;

		  if (fwd) {

			  step(volume, mesh, p.p4, steps, 1, index);

		  }
		  if (bwd) {

			  step(volume, mesh, p.p4, steps, -1, index);

		  }*/

		  //LogInfo("seedpoint:" << *it);

		  if (fwd) {

			  step(volume, mesh, *it, steps, 1, index);

		  }
		  if (bwd) {

			  step(volume, mesh, *it, steps, -1, index);

		  }
	  }
 

  outportline_.setData(mesh);
}

void FieldLine::step(const VolumeRAM *volume, BasicMesh *mesh,
                     const vec3 &startPos, const size_t &steps, const int &dir,
                     IndexBufferRAM *idx_buffer) {
  vec3 currentPos = startPos;

  auto dim = volume->getDimensions();

  for (size_t i = 0; i < steps; i++) {
    if (currentPos.x < 0)
      break;
    if (currentPos.y < 0)
      break;
    if (currentPos.z < 0)
      break;
    if (currentPos.x > 1)
      break;
    if (currentPos.y > 1)
      break;
    if (currentPos.z > 1)
      break;

    if (currentPos.x > 1 - 1.0 / dim.x)
      break;
    if (currentPos.y > 1 - 1.0 / dim.y)
      break;
    if (currentPos.z > 1 - 1.0 / dim.z)
      break;

	

	//***************************** trilinear ***************************

	//vec3 velocity = trilinear(volume, currentPos);
 
	//************************* Runge_kutta_4 Order & Euler************************

    //std::vector<std::shared_ptr<Volume>> vecVol;

    //auto v = inportvol_.getData()->clone();

    //vecVol.push_back(std::make_shared<Volume>(*v));

    //float current_time = (i * stepSize_.get()) > 1.f ? (i * stepSize_.get()) - 1  : i * stepSize_.get();

    float *vel = new float[3];

	vec3 perPos;

	if (integration_type_.get() == EULER) {
		perPos = euler(currentPos, volume, stepSize_.get(), dir, mat3(1.0), vel);
	}
	
	if (integration_type_.get() == RUNGE_KUTTA_4) {
		perPos = runge_kutta_4(currentPos, volume, stepSize_.get(), dir, mat3(1.0), vel);
	}

  // perPos = runge_kutta_4(currentPos, volume, stepSize_.get(), dir, mat3(1.0), vel);

    vec3 velocity = vec3(vel[0], vel[1], vel[2]);

    delete vel;

	 //*************************************

    vec4 color = velocity2color(velocity);

	//size_t idx1_ = mesh->addVertex(currentPos, currentPos, vec3(1), vec4(1.0f, 0.0f, 0.0f, 1.0f));

	size_t idx1_ = mesh->addVertex(perPos, perPos, vec3(1), color);

	size_t idx2_ = mesh->addVertex(currentPos, currentPos, vec3(1), color);
	
	//currentPos = currentPos + velocity.xyz() * stepSize_.get() * static_cast<float>(dir);

    idx_buffer->add(static_cast<uint32_t>(idx1_)); // Buffer
	idx_buffer->add(static_cast<uint32_t>(idx2_)); // Buffer

	currentPos = perPos;


  }
}

vec4 FieldLine::velocity2color(const vec3 &veloicty) {
  float d = glm::length(veloicty) * velocityScale_.get();

  minMaxVelocityTemp_.x = std::min(d, minMaxVelocityTemp_.x);
  minMaxVelocityTemp_.y = std::max(d, minMaxVelocityTemp_.y);

  const LayerRAM *tf = tf_.get().getData()->getRepresentation<LayerRAM>();
  if (d > 1)
    d = 1;
  if (d < 0)
    d = 0;
  uvec2 pos(d * (tf->getDimensions().x - 1), 0);

  return (vec4)tf->getValueAsVec4Double(pos);
}



std::vector<vec3> FieldLine::GetNeighbour(vec3 n, vec3 seedpoint, int m) {
	float fraction = 360 / m;
	float angle = 0;
	vec3 p1;
	std::vector<vec3> vec;

	vec3 w = glm::normalize(glm::cross(n, vec3(0, 0, 1)));
	for (size_t i = 0; i < m; i++){

		vec3 new_pos = seedpoint + d_.get() * w;
		w = glm::rotate(w, fraction, n);
		vec.push_back(new_pos);	
	}
	return vec;
}

/*float FieldLine::CalculateDistance(vec3& p1, const std::vector<vec3>& p2)
{
	float deltaX;
	for (auto p : p2) {
		 deltaX = glm::distance(p1, p);//square 
	}
	
	return deltaX;
}*/

} // namespace
