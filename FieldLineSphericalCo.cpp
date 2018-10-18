
#include "FieldLineSphericalCo.h"

#include <bitset>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/opengl/image/layergl.h>
#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/core/datastructures/image/layerram.h>

#define FWD 1
#define BWD 2
#define BOTH 3

#define EULER 1
#define RUNGE_KUTTA_4 2

#define FTLE_LOW_RESOLUTION 1
#define FTLE_HIGH_RESOLUTION 2
#define DOT 3

#define INDEX 1
#define FTLE 2
#define LENGTH 3 
#define Dot 4


namespace inviwo {

	static vec3 cartesianToSpherical(vec3 cartesianpos){

		float r = (sqrt(pow(cartesianpos.x, 2) + pow(cartesianpos.y, 2) + pow(cartesianpos.z, 2)));
		//float theta = -((std::acos(cartesianpos.z / r)) - (M_PI / 2)); //theta -90° to  90°
		float theta = std::atan2(cartesianpos.z, sqrt(pow(cartesianpos.x, 2) + pow(cartesianpos.y, 2)));
		float phi = std::atan2(cartesianpos.y, cartesianpos.x);//phi 0° to 360°

		vec3 sphericalCoordinate(r, theta, phi);
		return sphericalCoordinate;

	}

	static vec3 sphericalToCartesian(vec3 sphericalpos){

		float r = sphericalpos.x;
		float theta = (sphericalpos.y) /*+ (M_PI / 2)*/;
		float phi = (sphericalpos.z);
		
		/*float x = glm::sin(theta) * glm::cos(phi);
		float y = glm::sin(theta) * glm::sin(phi);
		float z = glm::cos(theta);*/

		float x = glm::cos(theta) * glm::cos(phi);
		float y = glm::cos(theta) * glm::sin(phi);
		float z = glm::sin(theta);
		
		vec3 cartesianCoordinate(x, y, z);
		return cartesianCoordinate * r;

	}

	/*static vec3 UnitVector_Cartesian(vec3 v, vec3 p_spherical){

		glm::mat3 A(
			cos(p_spherical.y)*cos(p_spherical.z), -sin(p_spherical.y)*cos(p_spherical.z), -sin(p_spherical.z),
			cos(p_spherical.y)*sin(p_spherical.z), -sin(p_spherical.y)*sin(p_spherical.z),  cos(p_spherical.z),
			                   sin(p_spherical.y),                     cos(p_spherical.y),                  0);

		glm::mat3 A_TP = glm::transpose(A);
		v = v * A_TP;
		return v;

	}*/

	static vec3 linear(const vec3 &a, const vec3 &b, const float &x) {
		return a + (b - a) * x;
	}
	static vec3 bilinear(const vec3 *samples, const vec2 &xy) {
		return linear(linear(samples[0], samples[1], xy.x),
			linear(samples[2], samples[3], xy.x), xy.y);
	}
	static vec3 trilinear(const VolumeRAM *volume, const vec3 &pos) {

		vec3 pos1 = pos;
		//pos1 = cartesianToSpherical(sphericalToCartesian(pos1));

		/*float theta = pos1.y;
		if (theta < -(M_PI / 2) || theta > (M_PI / 2)){
			LogInfoCustom("", theta);
		}
		if (cos(theta) < 0){
			LogErrorCustom("", "This shouldn't happen " << theta << " " << cos(theta));
		}*/

		if (pos1.y < -(M_PI / 2)){//theta -90° to  90°
			pos1.y = M_PI - glm::abs(pos1.y);
			pos1.z = pos1.z + M_PI;
		}
		if (pos1.y > (M_PI / 2)){
			pos1.y = M_PI - pos1.y;
			pos1.z = pos1.z - M_PI;
		}

		if (pos1.z < 0)//phi 0° to  360°
			pos1.z = pos1.z + (2 * M_PI);
		if (pos1.z > (2 * M_PI))
			pos1.z = pos1.z - (2 * M_PI);

		//betweem 0 and 1
		pos1.x = (pos1.x) / 29;//r [1-30]
		pos1.y = (pos1.y / M_PI) + 0.5;//theta [(-3.14159)-(3.14159)]
		pos1.z = pos1.z / (2 * M_PI);//phi [0-6.28319]

		float dummy;
		pos1.y = std::modf(pos1.y,&dummy );
		pos1.z = std::modf(pos1.z, &dummy);

		if (pos1.x < 0)
			return vec3(0, 0, 0);
		if (pos1.x > 1)
			return vec3(0, 0, 0);

		if (pos1.y < 0){
			pos1.y = 1 + (pos1.y);
		}
		if (pos1.y > 1){
			pos1.y = (pos1.y) - 1;
		}

		if (pos1.z < 0){
			pos1.z = 1 + (pos1.z);
		}
		if (pos1.z > 1){
			pos1.z = (pos1.z) - 1;
		}

		/*if (pos.y > 1)
			pos.y = 1;
			if (pos.y < 0)
			pos.y = 0;
			if (pos.z > 1)
			pos.z = 1;
			if (pos.z < 0)
			pos.z = 0;*/

		vec3 dim = vec3(volume->getDimensions() - size3_t(1));
		size3_t samplePos = size3_t(pos1 * dim);
		vec3 samples[8];

		vec3 interpolants = (pos1 * dim) - vec3(samplePos);

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

		//Scale the vector components in spherical coordinate d_r, d_phi, d_theta
		//v.x = (v.x);//r
		//v.y = (v.y) / (pos.x);//theta
		////v.z = (v.z) /((pos.x) * cos(theta));//phi
		//v.z = (v.z) / ((pos.x) * cos(pos.y));//phi

		v.x = (v.x);//r
		v.y = (v.y) * (pos.x);//theta
		v.z = (v.z) * (pos.x) * cos(pos.y);

		//v = UnitVector_Cartesian(v, pos1);
		//v = cartesianToSpherical(v);
		
		return v;
	}

	ProcessorClassIdentifier(FieldLineSphericalCo, "org.inviwo.FieldLineSphericalCo")
	ProcessorDisplayName(FieldLineSphericalCo, "FieldLineSphericalCo")
	ProcessorTags(FieldLineSphericalCo, Tags::None);
	ProcessorCategory(FieldLineSphericalCo, "Vector Field Visualization");
	ProcessorCodeState(FieldLineSphericalCo, CodeState::Experimental);

	FieldLineSphericalCo::FieldLineSphericalCo()
		: Processor(),
		inportvol_("vectorvolume"),
		inportpoint_("PointCloud"),
		inportface_("faceinport"),
		outportmesh_("linesStripsMesh_"),
		mesh_inport_("mesh_inport_"),
		outportline_("meshline"),


		d_Neighbor("d_", "Neighbor Distance", 0.110, 0.001, 1.0, 0.001),
		d_NextPoint("dd_", "NextPoint Distance ", 1.0, 0.1, 1.0, 0.1),
		d_trim_("d_trim_", "Trim Distance", 1.0, 0.1, 1.0, 0.1),
		Power_("Power_", "Power Number", 1.0, 0.1, 1.0, 0.1),
		//m_("m_", "neighbor points", 20, 0, 4, 1),
		numberOfSteps_("steps", "Step Number", 100, 1, 1000),
		stepSize_("stepSize", "Step Size", 0.001f, 0.0001f, 1.0f),
		stepDirection_("stepDirection", "Step Direction"),
		integration_type_("integration_type", "Integration"),
		colorvertex_("colorvertex", "Vertex Color"),
		colorline_("colorline", "Line Color"),
		tf_("transferFunction_", "Transfer Function"),
		velocityScale_("velocityScale_", "Velocity Scale", 1, 0, 2),
		maxlambda_("maxlambda", "Max Lambda", 100.0f, 1.0f, 10.0f, 1.0f),
		center_("center_", "center", vec3(0.0f), vec3(0.0f), vec3(0.0f)),
		spherical_pro_("spherical_pro_", "Cartesian/Spherical Coordinate", false)


	{
		addPort(inportvol_);
		addPort(inportpoint_);
		addPort(outportmesh_);
		addPort(mesh_inport_);
		addPort(outportline_);
		addPort(inportface_);

		stepSize_.setIncrement(0.0001f);

		stepDirection_.addOption("fwd", "Forward", FWD);
		stepDirection_.addOption("bwd", "Backwards", BWD);
		stepDirection_.addOption("bi", "Bi Directional", BOTH);

		integration_type_.addOption("euler", "Euler", EULER);
		integration_type_.addOption("rk4", " Runge-Kutta 4th Order", RUNGE_KUTTA_4);

		colorvertex_.addOption("FTLE_LR", "FTLE_LR", FTLE_LOW_RESOLUTION);
		colorvertex_.addOption("FTLE_HR", "FTLE_HR", FTLE_HIGH_RESOLUTION);
		colorvertex_.addOption("DOT", "DOT", DOT);

		colorline_.addOption("INDEX", "INDEX", INDEX);
		colorline_.addOption("FTLE", "FTLE", FTLE);
		colorline_.addOption("LENGTH", "LENGTH", LENGTH);
		colorline_.addOption("DOT", "DOT", Dot);

		colorvertex_.onChange(this, &FieldLineSphericalCo::updateColorLine);
		
		stepSize_.setCurrentStateAsDefault();
		stepDirection_.setCurrentStateAsDefault();

		addProperty(d_Neighbor);
		addProperty(d_NextPoint);
		addProperty(d_trim_);
		addProperty(Power_);
		addProperty(maxlambda_);
		addProperty(center_);
		addProperty(spherical_pro_);
		addProperty(numberOfSteps_);
		addProperty(stepSize_);
		addProperty(stepDirection_);
		addProperty(integration_type_);
		addProperty(colorvertex_);
		addProperty(colorline_);
		addProperty(tf_);
		addProperty(velocityScale_);

		tf_.get().clearPoints();
		tf_.get().addPoint(vec2(0, 1), vec4(0, 0, 1, 1));
		tf_.get().addPoint(vec2(0.5, 1), vec4(1, 1, 0, 1));
		tf_.get().addPoint(vec2(1, 1), vec4(1, 0, 0, 1));

		tf_.setCurrentStateAsDefault();

		/*LogInfo(sphericalToCartesian((vec3(1, 0, 0 + 2 * (M_PI)))));
		LogInfo(sphericalToCartesian((vec3(1, 0, M_PI / 2 + 2 * (M_PI)))));
		LogInfo(sphericalToCartesian((vec3(1, 0, M_PI + 2 * (M_PI)))));
		LogInfo(sphericalToCartesian((vec3(1, M_PI / 2, 0 + 2 * (M_PI)))));
		LogInfo(sphericalToCartesian((vec3(1, -(M_PI / 2), 0 + 2 * (M_PI)))));
		LogInfo(sphericalToCartesian((vec3(1, 0, (3 * (M_PI) / 2) + 2 * (M_PI)))));*/
	}

	FieldLineSphericalCo::~FieldLineSphericalCo() {}

	void FieldLineSphericalCo::updateColorLine(){

		if((colorvertex_.get() == FTLE_LOW_RESOLUTION) || (colorvertex_.get() == FTLE_HIGH_RESOLUTION)){
			colorline_.clearOptions();
			colorline_.addOption("INDEX", "INDEX", INDEX);
			colorline_.addOption("FTLE", "FTLE", FTLE);
			colorline_.addOption("LENGTH", "LENGTH", LENGTH);
		}
		if (colorvertex_.get() == DOT){
			colorline_.removeOption(1);
			colorline_.addOption("DOT", "DOT", Dot);	
		}
	}
	vec3 FieldLineSphericalCo::euler(vec3& p, const VolumeRAM* vector_field, float& stepsize, const int& dir,
		mat3& transformation_matrix, float* velocity) {

		vec3 K1 = trilinear(vector_field, p);
		K1 = K1 * stepsize;
		vec3 vel = transformation_matrix * K1;
		if (velocity != nullptr) {
			velocity[0] = vel.x;
			velocity[1] = vel.y;
			velocity[2] = vel.z;
		}
		vel.x = vel.x *  static_cast<float>(dir);
		return p + vel /**  static_cast<float>(dir)*/;
	}

	vec3 FieldLineSphericalCo::runge_kutta_4(vec3& p, const VolumeRAM*  vector_field, float& stepsize,
		const int& dir, mat3& transformation_matrix, float* velocity) {

		float h = stepsize / 2.f;
		vec3 K1 = trilinear(vector_field, p);
		K1 = K1 * stepsize;
		vec3 new_pos = p + h * (transformation_matrix * K1);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.y = (new_pos.y - 1); }

		vec3 K2 = trilinear(vector_field, new_pos);
		K2 = K2 * stepsize;
		new_pos = p + h * (transformation_matrix * K2);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.y = (new_pos.y - 1); }
		vec3 K3 = trilinear(vector_field, new_pos);

		K3 = K3 * stepsize;
		new_pos = p + stepsize * (transformation_matrix * K3);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.y = (new_pos.y - 1); }

		vec3 K4 = trilinear(vector_field, new_pos);
		K4 = K4 * stepsize;
		vec3 vel = (transformation_matrix * K1) * (1.f / 6.f) + (transformation_matrix * K2) * (1.f / 3.f) +
			(transformation_matrix * K3) * (1.f / 3.f) + (transformation_matrix * K4) * (1.f / 6.f);
		if (velocity != nullptr) {
			velocity[0] = vel.x;
			velocity[1] = vel.y;
			velocity[2] = vel.z;
		}
		return p + vel * static_cast<float>(dir);
	}

	void FieldLineSphericalCo::process() {

		BasicMesh *mesh = new BasicMesh();	
		if (spherical_pro_){
			mesh->setWorldMatrix(inportvol_.getData()->getWorldMatrix());
		}
		else{
			mesh->setModelMatrix(inportvol_.getData()->getModelMatrix());
			mesh->setWorldMatrix(inportvol_.getData()->getWorldMatrix());
		}
		auto inputvolume = inportvol_.getData();
		auto dims = inputvolume->getDimensions();
		auto seedpoints = inportpoint_.getData().get();
		auto faces = inportface_.getData().get();
		const VolumeRAM *volume =
			inportvol_.getData()->getRepresentation<VolumeRAM>();

	/*	mat4 index_to_texture =
			inputvolume->getCoordinateTransformer().getIndexToTextureMatrix();
		mat4 model_to_texture =
			inputvolume->getCoordinateTransformer().getModelToTextureMatrix();*/

		Calculate4Neighbor(*seedpoints, volume, mesh);
		CalculateVertexColor(*faces, *seedpoints, volume, mesh);
		ColorLines(mesh);

	}

	void FieldLineSphericalCo::Calculate4Neighbor(std::vector<vec3> points, const VolumeRAM *volume, BasicMesh *mesh){

		lines_.clear();
		index_.clear();
		vel_.clear();

		auto MeshToModelMatrix = mesh->getModelMatrix();
		size_t steps = numberOfSteps_.get();

		/*bool fwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(0); // False
		bool bwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(1); // True
		if (fwd && bwd) { // go both way
			steps /= 2;
		}*/

		size_t idx = 0;
		for (auto it = points.begin(); it != points.end(); it++) {

			vec3 pos = *it;
			vec3 pos_cartesian = (MeshToModelMatrix * vec4(pos, 1)).xyz;
			vec3 pos_spherical = cartesianToSpherical(pos_cartesian);
			vec3 start_vel = trilinear(volume, pos_spherical);
			vec3 N = glm::normalize(pos_cartesian);//Normal surface	
			float vel = start_vel.x;
			vel_.push_back(vel);

			 /*posspherical = (r, theta, phi)
			 N = (d_r, d_theta, d_phi)
			 vec3 N = UnitVector(start_vel, pos_spherical);*/

			/*if (glm::length(N) < 0.000001f){

				idx++;
				continue;
				}*/

			vec3 w = glm::normalize(glm::cross(N, vec3(0, 0, 1)));
			auto temp = std::abs(glm::dot(glm::normalize(N), vec3(0, 0, 1))) - 1;
			if (std::abs(temp) < 0.000001f) {
				w = glm::normalize(glm::cross(N, vec3(0, 1, 0)));
			}
			vec3 a = glm::normalize(glm::cross(N, w));
			auto model_matrix = mesh_inport_.getData()->getModelMatrix();
			vec3 p1 = (model_matrix * glm::vec4(*it, 1)).xyz;

			if (colorvertex_.get() == FTLE_LOW_RESOLUTION) {

				for (size_t i = 0; i < 5; i++) {

					Line l;
					vec3 p;

					if (i == 0) {
						p = p1;
					}
					if (i == 1) {
						p = p1 + d_Neighbor.get() * w;
					}
					if (i == 2) {
						p = p1 - d_Neighbor.get() * w;
					}
					if (i == 3) {
						p = p1 + d_Neighbor.get() * a;
					}
					if (i == 4) {
						p = p1 - d_Neighbor.get() * a;
					}

					//TODO transform Volume into texture space
					//mat4 p_to_texture = inportvol_.getData()->getCoordinateTransformer().getModelToTextureMatrix();
					//p = (p_to_texture * vec4(p, 1)).xyz;

					p = cartesianToSpherical(p);

					if (vel >= 0){
						step(volume, mesh, p, steps, 1, &l);
					}
					if (vel < 0) {
						step(volume, mesh, p, steps, -1, &l);
					}

					/*if (fwd){
						step(volume, mesh, p, steps, 1, &l);
						}
						if (bwd ) {
						step(volume, mesh, p, steps, -1, &l);
						}*/

					l.CalculateLength();
					//LogInfo("Line Length befor =" << l.GetLength());
					if (l.GetLength() > d_trim_.get()){
						l.Trim(d_trim_.get());
					}

					lines_.push_back(l);
				}
			}

			if ((colorvertex_.get() == FTLE_HIGH_RESOLUTION) || (colorvertex_.get() == DOT)) {

				Line l;
				p1 = cartesianToSpherical(p1);

				if (vel >= 0){
					step(volume, mesh, p1, steps, 1, &l);
				}
				if (vel < 0) {
					step(volume, mesh, p1, steps, -1, &l);
				}

				l.CalculateLength();
				if (l.GetLength() > d_trim_.get()){
					l.Trim(d_trim_.get());
				}
				lines_.push_back(l);
			}

			index_.push_back(idx);
			idx++;
		}
	}

	void FieldLineSphericalCo::CalculateVertexColor(std::vector<Face> faces, std::vector<vec3> points, const VolumeRAM *volume, BasicMesh *mesh){

		//float max1 = 0.0f;
		auto inMesh = mesh_inport_.getData();
		auto meshdata = dynamic_cast<const BasicMesh*>(inMesh.get());
		if (meshdata == nullptr)
			return;
		auto Mesh_sphere = meshdata->clone();
		if (colorvertex_.get() == FTLE_LOW_RESOLUTION) {
			FTLE_Low_Resolution(Mesh_sphere);
		}
		if (colorvertex_.get() == FTLE_HIGH_RESOLUTION) {
			FTLE_High_Resolution(faces, Mesh_sphere);
		}
		if (colorvertex_.get() == DOT) {
			Dot_Posnor_Vec(Mesh_sphere);
		}
		Mesh_sphere->getEditableRepresentation<MeshRAM>();

		outportmesh_.setData(Mesh_sphere);
		outportline_.setData(mesh);

	}


	void FieldLineSphericalCo::ColorLines(BasicMesh *mesh){

		auto IndexSeedPoint = index_.begin();
		size_t IndexSeedPointEnd = index_.back();
		auto C_FTLE = colorFTLE_.begin();
		auto C_dot = colorDot_.begin();
	
		if (colorvertex_.get() == FTLE_LOW_RESOLUTION) {

			for (auto it = lines_.begin(); it != lines_.end(); it += 5) {
				if (colorline_.get() == INDEX){
					float div = float(float(*IndexSeedPoint) / float(IndexSeedPointEnd));
					vec4 color_Index = idx2color(div);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_Index);
						(it + 1)->AddToMeshSphericalToCartesian(mesh, color_Index);
						(it + 2)->AddToMeshSphericalToCartesian(mesh, color_Index);
						(it + 3)->AddToMeshSphericalToCartesian(mesh, color_Index);
						(it + 4)->AddToMeshSphericalToCartesian(mesh, color_Index);
					}
					else{
						it->AddToMesh(mesh, color_Index);
						(it + 1)->AddToMesh(mesh, color_Index);
						(it + 2)->AddToMesh(mesh, color_Index);
						(it + 3)->AddToMesh(mesh, color_Index);
						(it + 4)->AddToMesh(mesh, color_Index);
					}
					IndexSeedPoint++;
				}
				if (colorline_.get() == FTLE){
					vec4 color_FTLE = vec4(*C_FTLE);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_FTLE);
						(it + 1)->AddToMeshSphericalToCartesian(mesh, color_FTLE);
						(it + 2)->AddToMeshSphericalToCartesian(mesh, color_FTLE);
						(it + 3)->AddToMeshSphericalToCartesian(mesh, color_FTLE);
						(it + 4)->AddToMeshSphericalToCartesian(mesh, color_FTLE);
					}
					else{
						it->AddToMesh(mesh, color_FTLE);
						(it + 1)->AddToMesh(mesh, color_FTLE);
						(it + 2)->AddToMesh(mesh, color_FTLE);
						(it + 3)->AddToMesh(mesh, color_FTLE);
						(it + 4)->AddToMesh(mesh, color_FTLE);
					}
					C_FTLE++;
				}
				if (colorline_.get() == LENGTH){
					it->CalculateLength();
					float L = it->GetLength();
					vec4 color_Length = idx2color(L);

					(it + 1)->CalculateLength();
					float L1 = (it + 1)->GetLength();
					vec4 color_Length1 = idx2color(L1);

					(it + 2)->CalculateLength();
					float L2 = (it + 2)->GetLength();
					vec4 color_Length2 = idx2color(L2);

					(it + 3)->CalculateLength();
					float L3 = (it + 3)->GetLength();
					vec4 color_Length3 = idx2color(L3);

					(it + 4)->CalculateLength();
					float L4 = (it + 4)->GetLength();
					vec4 color_Length4 = idx2color(L4);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_Length);
						(it + 1)->AddToMeshSphericalToCartesian(mesh, color_Length1);
						(it + 2)->AddToMeshSphericalToCartesian(mesh, color_Length2);
						(it + 3)->AddToMeshSphericalToCartesian(mesh, color_Length3);
						(it + 4)->AddToMeshSphericalToCartesian(mesh, color_Length4);
					}
					else{
						it->AddToMesh(mesh, color_Length);
						(it + 1)->AddToMesh(mesh, color_Length1);
						(it + 2)->AddToMesh(mesh, color_Length2);
						(it + 3)->AddToMesh(mesh, color_Length3);
						(it + 4)->AddToMesh(mesh, color_Length4);
					}
				}
			}
		}

		if ((colorvertex_.get() == FTLE_HIGH_RESOLUTION)) {

			for (auto it = lines_.begin(); it != lines_.end(); it++) {
				if (colorline_.get() == INDEX){
					float div = float(float(*IndexSeedPoint) / float(IndexSeedPointEnd));
					vec4 color_Index = idx2color(div);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_Index);
					}
					else{
						it->AddToMesh(mesh, color_Index);
					}
					IndexSeedPoint++;
				}
				if (colorline_.get() == FTLE){
					vec4 color_FTLE = vec4(*C_FTLE);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_FTLE);
					}
					else{
						it->AddToMesh(mesh, color_FTLE);
					}
					C_FTLE++;
				}
				if (colorline_.get() == LENGTH){
					it->CalculateLength();
					float L = it->GetLength();
					vec4 color_Length = idx2color(L);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_Length);
					}
					else{
						it->AddToMesh(mesh, color_Length);
					}
				}
			
			}
		}
		if ((colorvertex_.get() == DOT)) {

			for (auto it = lines_.begin(); it != lines_.end(); it++) {

				if (colorline_.get() == INDEX){
					float div = float(float(*IndexSeedPoint) / float(IndexSeedPointEnd));
					vec4 color_Index = idx2color(div);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_Index);
					}
					else{
						it->AddToMesh(mesh, color_Index);
					}
					IndexSeedPoint++;
				}
				if (colorline_.get() == LENGTH){
					it->CalculateLength();
					float L = it->GetLength();
					vec4 color_Length = idx2color(L);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_Length);
					}
					else{
						it->AddToMesh(mesh, color_Length);
					}
				}
				if (colorline_.get() == Dot){
					vec4 color_dot = vec4(*C_dot);
					if (spherical_pro_){
						it->AddToMeshSphericalToCartesian(mesh, color_dot);
					}
					else{
						it->AddToMesh(mesh, color_dot);
					}
					C_dot++;
				}
			}
		}
	}

	void FieldLineSphericalCo::step(const VolumeRAM *volume, BasicMesh *mesh,
		const vec3 &startPos, const size_t &steps, const int &dir, Line *l) {

		vec3 currentPos = startPos;
		l->AddPoint(startPos);

		IndexBufferRAM *idx_buffer =
			mesh->addIndexBuffer(DrawType::LINES, ConnectivityType::STRIP); // Buffer

		int i = 0;
		auto dim = volume->getDimensions();

	    for (size_t i = 0; i < steps; i++) {

		//while (true){

			if (currentPos.x < 1)//r 1 to  30
                break;
			if (currentPos.x > 30)
				break;

			vec3 vel;
			vec3 perPos;

			//auto m = glm::inverse(inportvol_.getData()->getBasis()); //TODO change mat3 to inv basis from volume (to convert to texture space=
			if (integration_type_.get() == EULER) {
				perPos = euler(currentPos, volume, stepSize_.get(), dir, mat3(1), glm::value_ptr(vel));
			}
			if (integration_type_.get() == RUNGE_KUTTA_4) {
				perPos = runge_kutta_4(currentPos, volume, stepSize_.get(), dir, mat3(1), glm::value_ptr(vel));
			}
			if (glm::length(vel) < 0.000001f){ //vel terminate
				break;
			}

			/*if (i++ % 100000000 == 0){
				std::cout << currentPos << "  " << vel << std::endl;
				break;
			}*/

			l->AddPoint(perPos);
			currentPos = perPos;

		}
	}

	void FieldLineSphericalCo::FTLE_Low_Resolution(BasicMesh *mesh_sphere){

		vec4 color_LR_fwd;
		vec4 color_LR_bwd;

		float maxlbda = 0.0f;
		float minlbda = 0.0f;
		colorFTLE_.clear();

		auto IndexSeedPoint = index_.begin();
		auto velocity = vel_.begin();

		for (auto it = lines_.begin(); it != lines_.end(); it += 5) {

			vec3 e1 = (sphericalToCartesian((it + 1)->GetLastPoint()) - sphericalToCartesian((it + 2)->GetLastPoint())) / (d_Neighbor.get() * 2);
			vec3 e2 = (sphericalToCartesian((it + 3)->GetLastPoint()) - sphericalToCartesian((it + 4)->GetLastPoint())) / (d_Neighbor.get() * 2);

			glm::mat2x3 A = glm::mat2x3(e1, e2);
			glm::mat3x2 AT = glm::transpose(A);
			auto e = util::glm2eigen(A);
			Eigen::EigenSolver<Eigen::MatrixXf> es(e.transpose()*e);

			std::complex<double> lambda1 = es.eigenvalues()[0];
			std::complex<double> lambda2 = es.eigenvalues()[1];
			std::complex<double> lambda3 = es.eigenvalues()[2];

			lambda1 = std::abs(lambda1);
			lambda2 = std::abs(lambda2);
			lambda3 = std::abs(lambda3);

			if (lambda1.real() > 0){
				lambda1 = std::log(lambda1);
			}
			if (lambda2.real() > 0){
				lambda2 = std::log(lambda2);
			}
			if (lambda3.real() > 0){
				lambda3 = std::log(lambda3);
			}
		
			/*vec3 p = it->GetPoints()->at(0);
			p = glm::normalize(sphericalToCartesian(p));
			p = cartesianToSpherical(p);
			color.r = glm::clamp(p.x, 0.f, 1.f);

			color.r = p.x;
			color.g = (p.y / 0.3f) / (M_PI); //theta 0° to  180°
			color.b = (p.y / 0.3f) / (M_PI); //phi 0° to  360°
			color.a = (p.y / 0.3f) / (M_PI);*/

			double Max_local_lambda = std::max(lambda1.real(), lambda2.real());
			Max_local_lambda = std::max(lambda3.real(), Max_local_lambda);

			if (Max_local_lambda < minlbda){
				minlbda = Max_local_lambda;
			}
			if (Max_local_lambda > maxlbda){
				maxlbda = Max_local_lambda;
			}

			Max_local_lambda = std::abs(Max_local_lambda);

			vec4 color_LR;
			 //red
			 color_LR_bwd = vec4(std::pow(float(Max_local_lambda / maxlambda_.get()), Power_), 0, 0, 1);
			 //green
			 color_LR_fwd = vec4(0,
				                 std::pow(float(Max_local_lambda / maxlambda_.get()), Power_),
								 0, 1);
			
			//mesh_sphere->setVertexColor(*IndexSeedPoint, color_LR_fwd);
				
			if (*velocity >= 0){
				color_LR = color_LR_fwd;
				mesh_sphere->setVertexColor(*IndexSeedPoint, color_LR);
			   }
			if (*velocity < 0){
				color_LR = color_LR_bwd;
				mesh_sphere->setVertexColor(*IndexSeedPoint, color_LR);
			   }

			colorFTLE_.push_back(color_LR);
	
			IndexSeedPoint++;
			velocity++;
		}

		LogInfo(" maximumlambda LR = " << maxlbda);
		LogInfo(" minimumlambda LR = " << minlbda);
	}

	/*bool cmpFloat(const float &a, const float &b){
		return a < b;
	}*/

	struct Wrapper{
		int idx;
		vec3 Pos;
		vec3 TP;
		float angle;
		//Face f;
	};

	bool cmp_idx(const Wrapper &a, const Wrapper &b){
		return a.idx < b.idx;
	}

	bool cmp_angle(const Wrapper &a, const Wrapper &b){
		return a.angle < b.angle;
	}

	void FieldLineSphericalCo::FTLE_High_Resolution(std::vector<Face> faces, BasicMesh *mesh_sphere){

		float maxlbda = 0.0f;
		float minlbda = 0.0f;

		vec4 color_HR_fwd;
		vec4 color_HR_bwd;
		colorFTLE_.clear();

		auto velocity = vel_.begin();

		std::vector<Face> two_face;
		std::vector<Face> two_opposit_faces;
		std::set<size_t> points_around_sp;
		std::vector<Wrapper> points_around_sp_Wrapper;
		std::vector<size_t> ln;
		std::vector<size_t> tmppoint2;
		std::vector<Line> line;
		std::vector<Face> faces_have_sp_sort;
		std::vector<size_t> points_around_sp_sort;

		auto IndexSeedPoint = index_.begin();

		for (auto it = lines_.begin(); it != lines_.end(); it ++) {

			two_face.clear();
			points_around_sp.clear();
			ln.clear();
			line.clear();
			two_opposit_faces.clear();
			points_around_sp_Wrapper.clear();
			faces_have_sp_sort.clear();
			points_around_sp_sort.clear();
			
			Wrapper sp;
			sp.idx = *IndexSeedPoint;
			sp.Pos = lines_.at(*IndexSeedPoint).GetPoints()->at(0);
			sp.Pos = sphericalToCartesian(sp.Pos);

			for (auto f = faces.begin(); f != faces.end(); f++){//for all faces		
				if (sp.idx == f->p1){
					points_around_sp.insert(f->p2);
				    points_around_sp.insert(f->p3);			
						 }				
				if ((sp.idx == f->p2)){			
					points_around_sp.insert(f->p3);
					points_around_sp.insert(f->p1);			
					}		
				if ((sp.idx == f->p3)){						
					points_around_sp.insert(f->p1);
					points_around_sp.insert(f->p2);			
				}					
		    }
			for (auto i = points_around_sp.begin(); i != points_around_sp.end(); i++){
				Wrapper p;
				p.idx = *i;
				p.Pos =lines_.at(p.idx).GetPoints()->at(0);
				points_around_sp_Wrapper.push_back(p);

			}
		
			//LogInfo("Number of neighbor points: " << points_around_sp.size());

			/*for (auto i = points_around_sp_Wrapper.begin(); i != points_around_sp_Wrapper.end(); i++){
				auto pos = sphericalToCartesian(i->Pos);
				LogInfo("distance" << glm::distance(pos, sp.Pos)  << pos << i->Pos << glm::length(pos));
			}*/

			std::complex<double> lambda1;
			std::complex<double> lambda2;
			std::complex<double> lambda3;
			std::vector<float> angle;			
			double Max_local_lambda;

			if (points_around_sp.size() == 5){

				std::vector<Wrapper> points_around_sp_Wrapper_tmp1(points_around_sp_Wrapper);

				auto p1 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p2 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p3 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p4 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p5 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();

				p1.Pos = sphericalToCartesian(p1.Pos);
				p2.Pos = sphericalToCartesian(p2.Pos);
				p3.Pos = sphericalToCartesian(p3.Pos);
				p4.Pos = sphericalToCartesian(p4.Pos);
				p5.Pos = sphericalToCartesian(p5.Pos);

				auto n = glm::normalize(sp.Pos);
				auto T1 = glm::mat3(glm::normalize(vec3(p1.Pos - sp.Pos)), glm::cross(glm::normalize(vec3(p1.Pos - sp.Pos)),n), n);
				T1 = glm::inverse(T1);

				//To project a 3d point to a 3d plane 
				//1.distance from point to plane along the normal and Multiply the unit normal vector by the distance. 
				p1.Pos -= glm::dot(p1.Pos - sp.Pos, n) * n;
				p2.Pos -= glm::dot(p2.Pos - sp.Pos, n) * n;
				p3.Pos -= glm::dot(p3.Pos - sp.Pos, n) * n;
				p4.Pos -= glm::dot(p4.Pos - sp.Pos, n) * n;
				p5.Pos -= glm::dot(p5.Pos - sp.Pos, n) * n;

				//2.subtract that vector from your point.
				p1.Pos -= sp.Pos;
				p2.Pos -= sp.Pos;
				p3.Pos -= sp.Pos;
				p4.Pos -= sp.Pos;
				p5.Pos -= sp.Pos;
			
				p1.TP = T1 * p1.Pos;
				p2.TP = T1 * p2.Pos;
				p3.TP = T1 * p3.Pos;
				p4.TP = T1 * p4.Pos;
				p5.TP = T1 * p5.Pos;
				sp.TP = T1 * sp.Pos;
					
				p1.angle = std::atan2((p1.TP.y ), (p1.TP.x ));
				p2.angle = std::atan2((p2.TP.y ), (p2.TP.x ));
				p3.angle = std::atan2((p3.TP.y ), (p3.TP.x ));
				p4.angle = std::atan2((p4.TP.y ), (p4.TP.x ));
				p5.angle = std::atan2((p5.TP.y ), (p5.TP.x ));

				/*static bool first = true;
				if (first){
					LogInfo("Angles:");
					LogInfo(p1.angle);
					LogInfo(p2.angle);
					LogInfo(p3.angle);
					LogInfo(p4.angle);
					LogInfo(p5.angle);
				}*/

				if (p1.angle < 0){
					p1.angle = (2 * (M_PI)) + p1.angle;
					}
				if (p2.angle < 0){
					p2.angle = (2 * (M_PI)) + p2.angle;
					}
				if (p3.angle < 0){
					p3.angle = (2 * (M_PI)) + p3.angle;
					}
				if (p4.angle < 0){
					p4.angle = (2 * (M_PI)) + p4.angle;
					}
				if (p5.angle < 0){
					p5.angle = (2 * (M_PI)) + p5.angle;
					}

				if (p1.angle > (4.5 * (2 * (M_PI) / 5))){
					p1.angle = 0;
				}
				if (p2.angle > (4.5 * (2 * (M_PI) / 5))){
					p2.angle = 0;
				}
				if (p3.angle > (4.5 * (2 * (M_PI) / 5))){
					p3.angle = 0;
				}
				if (p4.angle > (4.5 * (2 * (M_PI) / 5))){
					p4.angle = 0;
				}
				if (p5.angle > (4.5 * (2 * (M_PI) / 5))){
					p5.angle = 0;
				}
				
					points_around_sp_Wrapper.clear();

					points_around_sp_Wrapper.push_back(p1);
					points_around_sp_Wrapper.push_back(p2);
					points_around_sp_Wrapper.push_back(p3);
					points_around_sp_Wrapper.push_back(p4);
					points_around_sp_Wrapper.push_back(p5);

					std::sort(points_around_sp_Wrapper.begin(), points_around_sp_Wrapper.end(), cmp_angle);
	
               auto l1 = lines_[points_around_sp_Wrapper[0].idx];	
			   auto l2 = lines_[points_around_sp_Wrapper[1].idx];	
			   auto l3 = lines_[points_around_sp_Wrapper[2].idx];
			   auto l4 = lines_[points_around_sp_Wrapper[3].idx];
			   auto l5 = lines_[points_around_sp_Wrapper[4].idx];

			   float weight = 0.32491; // two vertices distance (1.05146) * sin(18)
				vec3 e1 = (((sphericalToCartesian((l1).GetLastPoint())) - ((sphericalToCartesian((l4).GetLastPoint()) + sphericalToCartesian((l3).GetLastPoint())) / 2.f)) / (2 * d_Neighbor.get()));

			    vec3 e2 = (((sphericalToCartesian((l5).GetLastPoint())) - ((sphericalToCartesian((l3).GetLastPoint()) * weight + sphericalToCartesian((l2).GetLastPoint()) * (1 - weight)) / 2.f)) / (2 * d_Neighbor.get()));

				/*vec3 e2 = (((sphericalToCartesian((l5).GetLastPoint())) - ((sphericalToCartesian((l3).GetLastPoint()) + sphericalToCartesian((l2).GetLastPoint())) / 2.f)) / (2 * d_Neighbor.get()));*/
				/*vec3 e2 = ((((sphericalToCartesian((l5).GetLastPoint()) + ((sphericalToCartesian((l5).GetLastPoint()) + sphericalToCartesian((l4).GetLastPoint())) / 2.f)) / 2.f) - ((sphericalToCartesian((l2).GetLastPoint()) + ((sphericalToCartesian((l2).GetLastPoint()) + sphericalToCartesian((l3).GetLastPoint())) / 2.f)) / 2.f)) / (2 * d_Neighbor.get()));*/
		
				glm::mat2x3 A = glm::mat2x3(e1, e2);
				glm::mat3x2 AT = glm::transpose(A);
				auto e = util::glm2eigen(A);
				Eigen::EigenSolver<Eigen::MatrixXf> es(e.transpose()*e);

				lambda1 = es.eigenvalues()[0];
				lambda2 = es.eigenvalues()[1];
				lambda3 = es.eigenvalues()[2];

				lambda1 = std::abs(lambda1);
				lambda2 = std::abs(lambda2);
				lambda3 = std::abs(lambda3);

				if (lambda1.real() > 0){
					lambda1 = std::log(lambda1);
				}
				if (lambda2.real() > 0){
					lambda2 = std::log(lambda2);
				}
				if (lambda3.real() > 0){
					lambda3 = std::log(lambda3);
				}

				/*vec3 p = it->GetPoints()->at(0);
				p = glm::normalize(sphericalToCartesian(p));
				p = cartesianToSpherical(p);
				color.r = glm::clamp(p.x, 0.f, 1.f);

				color.r = p.x;
				color.g = (p.y / 0.3f) / (M_PI); //theta 0° to  180°
				color.b = (p.y / 0.3f) / (M_PI); //phi 0° to  360°
				color.a = (p.y / 0.3f) / (M_PI);*/

				Max_local_lambda = std::max(lambda1.real(), lambda2.real());
				Max_local_lambda = std::max(lambda3.real(), Max_local_lambda);

				if (Max_local_lambda < minlbda){
					minlbda = Max_local_lambda;
				}
				if (Max_local_lambda > maxlbda){
					maxlbda = Max_local_lambda;
				}

				Max_local_lambda = std::abs(Max_local_lambda);

				
				//red
				color_HR_bwd = vec4(std::pow(float(Max_local_lambda / maxlambda_.get()), Power_), 0, 0, 1);
			    //green
				color_HR_fwd = vec4(0,
					                std::pow(float(Max_local_lambda / maxlambda_.get()), Power_), 
							        0, 1);

			}

			if (points_around_sp.size() == 6){

				std::vector<Wrapper> points_around_sp_Wrapper_tmp1(points_around_sp_Wrapper);

				auto p1 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p2 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p3 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p4 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p5 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();
				auto p6 = points_around_sp_Wrapper_tmp1.back();
				points_around_sp_Wrapper_tmp1.pop_back();

				p1.Pos = sphericalToCartesian(p1.Pos);
				p2.Pos = sphericalToCartesian(p2.Pos);
				p3.Pos = sphericalToCartesian(p3.Pos);
				p4.Pos = sphericalToCartesian(p4.Pos);
				p5.Pos = sphericalToCartesian(p5.Pos);
				p6.Pos = sphericalToCartesian(p6.Pos);

				auto n = glm::normalize(sp.Pos);
				auto T1 = glm::mat3(glm::normalize(vec3(p1.Pos - sp.Pos)), glm::cross(glm::normalize(vec3(p1.Pos - sp.Pos)), n), n);	
				T1 = glm::inverse(T1);

				p1.Pos -= glm::dot(p1.Pos - sp.Pos, n) * n;
				p2.Pos -= glm::dot(p2.Pos - sp.Pos, n) * n;
				p3.Pos -= glm::dot(p3.Pos - sp.Pos, n) * n;
				p4.Pos -= glm::dot(p4.Pos - sp.Pos, n) * n;
				p5.Pos -= glm::dot(p5.Pos - sp.Pos, n) * n;
				p6.Pos -= glm::dot(p6.Pos - sp.Pos, n) * n;

				p1.Pos -= sp.Pos;
				p2.Pos -= sp.Pos;
				p3.Pos -= sp.Pos;
				p4.Pos -= sp.Pos;
				p5.Pos -= sp.Pos;
				p6.Pos -= sp.Pos;
			
				p1.TP = T1* p1.Pos;
				p2.TP = T1* p2.Pos;
				p3.TP = T1* p3.Pos;
				p4.TP = T1* p4.Pos;
				p5.TP = T1* p5.Pos;
				p6.TP = T1* p6.Pos;
				sp.TP = T1* sp.Pos;

				p1.angle = std::atan2((p1.TP.y ), (p1.TP.x ));
				p2.angle = std::atan2((p2.TP.y ), (p2.TP.x ));
				p3.angle = std::atan2((p3.TP.y ), (p3.TP.x ));
				p4.angle = std::atan2((p4.TP.y ), (p4.TP.x ));
				p5.angle = std::atan2((p5.TP.y ), (p5.TP.x ));
				p6.angle = std::atan2((p6.TP.y ), (p6.TP.x ));

				if (p1.angle < 0){
					p1.angle = (2 * (M_PI)) + p1.angle;
				}
				if (p2.angle < 0){
					p2.angle = (2 * (M_PI)) + p2.angle;
				}
				if (p3.angle < 0){
					p3.angle = (2 * (M_PI)) + p3.angle;
				}
				if (p4.angle < 0){
					p4.angle = (2 * (M_PI)) + p4.angle;
				}
				if (p5.angle < 0){
					p5.angle = (2 * (M_PI)) + p5.angle;
				}
				if (p6.angle < 0){
					p6.angle = (2 * (M_PI)) + p6.angle;
				}

				if (p1.angle > (5.5 * (2 * (M_PI) / 6))){
					p1.angle =  0;
				}
				if (p2.angle > ( 5.5 * (2 * (M_PI) / 6))){
					p2.angle = 0;
				}
				if (p3.angle > (5.5 * (2 * (M_PI) / 6))){
					p3.angle = 0;
				}
				if (p4.angle > (5.5 * (2 * (M_PI) / 6))){
					p4.angle = 0;
				}
				if (p5.angle > (5.5 * (2 * (M_PI) / 6))){
					p5.angle = 0;
				}
				if (p6.angle > (5.5 * (2 * (M_PI) / 6))){
					p6.angle = 0;
				}

				points_around_sp_Wrapper.clear();

				points_around_sp_Wrapper.push_back(p1);
				points_around_sp_Wrapper.push_back(p2);
				points_around_sp_Wrapper.push_back(p3);
				points_around_sp_Wrapper.push_back(p4);
				points_around_sp_Wrapper.push_back(p5);
				points_around_sp_Wrapper.push_back(p6);

				std::sort(points_around_sp_Wrapper.begin(), points_around_sp_Wrapper.end(), cmp_angle);

                auto l1 = lines_[points_around_sp_Wrapper[0].idx];
				auto l2 = lines_[points_around_sp_Wrapper[1].idx];
				auto l3 = lines_[points_around_sp_Wrapper[2].idx];
				auto l4 = lines_[points_around_sp_Wrapper[3].idx];
				auto l5 = lines_[points_around_sp_Wrapper[4].idx];
				auto l6 = lines_[points_around_sp_Wrapper[5].idx];

				vec3 e1 = ((sphericalToCartesian((l1).GetLastPoint()) - sphericalToCartesian((l4).GetLastPoint())) / (2 * d_Neighbor.get()));
				vec3 e2 = (sphericalToCartesian((l6).GetLastPoint()) + sphericalToCartesian((l5).GetLastPoint())) / 2.f;
				vec3 e3 = (sphericalToCartesian((l2).GetLastPoint()) + sphericalToCartesian((l3).GetLastPoint())) / 2.f;
				vec3 e4 = (e2 - e3) / (2 * d_Neighbor.get());
				//vec3 e4 = ((vec3(sqrt(3)) / vec3(2)) * (e2 + e3)) / (2 * d_Neighbor.get());
						  
				glm::mat2x3 A = glm::mat2x3(e1, e4);
				glm::mat3x2 AT = glm::transpose(A);
			    auto e = util::glm2eigen(A);
				Eigen::EigenSolver<Eigen::MatrixXf> es(e.transpose()*e);

				lambda1 = es.eigenvalues()[0];
				lambda2 = es.eigenvalues()[1];
				lambda3 = es.eigenvalues()[2];

				lambda1 = std::abs(lambda1);
				lambda2 = std::abs(lambda2);
				lambda3 = std::abs(lambda3);

				if (lambda1.real() > 0){
					lambda1 = std::log(lambda1);
				}
				if (lambda2.real() > 0){
					lambda2 = std::log(lambda2);
				}
				if (lambda3.real() > 0){
					lambda3 = std::log(lambda3);
				}

				Max_local_lambda = std::max(lambda1.real(), lambda2.real());
				Max_local_lambda = std::max(lambda3.real(), Max_local_lambda);

				if (Max_local_lambda < minlbda){
					minlbda = Max_local_lambda;
				}
				if (Max_local_lambda > maxlbda){
					maxlbda = Max_local_lambda;
				}

				Max_local_lambda = std::abs(Max_local_lambda);

				//red
				color_HR_bwd = vec4(std::pow(float(Max_local_lambda / maxlambda_.get()), Power_), 0, 0, 1);
				//green
				color_HR_fwd = vec4(0,
					                std::pow(float(Max_local_lambda / maxlambda_.get()), Power_), 
					                0, 1);

			}

			//mesh_sphere->setVertexColor(*IndexSeedPoint, color_HR_fwd);

			vec4 color_HR;

			if (*velocity >= 0){
				color_HR = color_HR_fwd;
				mesh_sphere->setVertexColor(*IndexSeedPoint, color_HR);
			}
			if (*velocity < 0){
				color_HR = color_HR_bwd;
				mesh_sphere->setVertexColor(*IndexSeedPoint, color_HR);
			}

			colorFTLE_.push_back(color_HR);
			IndexSeedPoint++;
			velocity++;

		}

		LogInfo(" maximumlambda HR = " << maxlbda);
		LogInfo(" minimumlambda HR = " << minlbda);

	}

	void FieldLineSphericalCo::Dot_Posnor_Vec(BasicMesh *mesh_sphere){

		vec4 color_dot;
		auto IndexSeedPoint = index_.begin();

		colorDot_.clear();

		//if (colorvertex_.get() == FTLE_LOW_RESOLUTION) {
		//	for (auto it = lines_.begin(); it != lines_.end(); it += 5) {
		//		auto posnor = glm::normalize(it->GetPoints()->at(0));
		//		//posnor = sphericalToCartesian(posnor);
		//		auto vec = glm::normalize((it->GetPoints()->at(1)) - (it->GetPoints()->at(0)));
		//		auto ang = glm::dot(posnor, vec);
		//		if (ang > 0) {
		//			color_dot = vec4(0, 1 + ang, 0, 1); //green
		//		}
		//		else {
		//			color_dot = vec4(0, 0, 1 - ang, 1); //blue
		//		}
		//		mesh_sphere->setVertexColor(*IndexSeedPoint, color_dot);
		//		IndexSeedPoint++;
		//	}
		//}

			for (auto it = lines_.begin(); it != lines_.end(); it ++) {
				auto posnor = glm::normalize(it->GetPoints()->at(0));
				//posnor = sphericalToCartesian(posnor);
				auto vec = glm::normalize((it->GetPoints()->at(1)) - (it->GetPoints()->at(0)));
				auto ang = glm::dot(posnor, vec);
				if (ang > 0) {
					color_dot = vec4(0, 1 + ang, 0, 1); //green
				}
				else {
					color_dot = vec4(0, 0, 1 - ang, 1); //blue
				}
				mesh_sphere->setVertexColor(*IndexSeedPoint, color_dot);
				colorDot_.push_back(color_dot);
				IndexSeedPoint++;
			}
	}

	vec4 FieldLineSphericalCo::velocity2color(const vec3 &veloicty) {

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

	vec4 FieldLineSphericalCo::idx2color(float idx) {
		const LayerRAM *tf = tf_.get().getData()->getRepresentation<LayerRAM>();
		if (idx > 1)
			idx = 1;
		if (idx < 0)
			idx = 0;
		uvec2 pos(idx * (tf->getDimensions().x - 1), 0);
		return (vec4)tf->getValueAsVec4Double(pos);
	}

	float FieldLineSphericalCo::radianToDegree(float rad) {
		return rad * (180.f / M_PI);
	}

} // namespace
