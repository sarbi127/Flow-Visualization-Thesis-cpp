

#include "FieldLineSC.h"

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

namespace inviwo {

	static vec3 cartesianToSpherical(vec3 cartesianCoordinate){

		float r = (sqrt(pow(cartesianCoordinate.x, 2) + pow(cartesianCoordinate.y, 2) + pow(cartesianCoordinate.z, 2)));

		float theta = (std::acos(cartesianCoordinate.z / r)) / M_PI; //0° to  180°

		float phi = (std::atan2(cartesianCoordinate.y, cartesianCoordinate.x)) / (2 * M_PI);//0° to 360°

		if (phi < 0){

			phi = 1 + (phi);
		}

		r = (r) / (5.3);

		vec3 sphericalCoordinate(r, theta, phi);

		return sphericalCoordinate;

	}

	static vec3 sphericalToCartesian(vec3 sphericalCoordinate){

		float theta = (sphericalCoordinate.y) * M_PI;
		float phi = (sphericalCoordinate.z) * (2 * M_PI);

		float x = glm::sin(theta) * glm::cos(phi);

		float y = glm::sin(theta) * glm::sin(phi);

		float z = glm::cos(theta);

		float r = sphericalCoordinate.x;

		r = (r * (5.3));

		vec3 cartesianCoordinate(x, y, z);

		return cartesianCoordinate * r;

	}

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

	ProcessorClassIdentifier(FieldLineSC, "org.inviwo.FieldLineSC")
		ProcessorDisplayName(FieldLineSC, "FieldLineSC")
		ProcessorTags(FieldLineSC, Tags::None);
	ProcessorCategory(FieldLineSC, "Vector Field Visualization");
	ProcessorCodeState(FieldLineSC, CodeState::Experimental);

	FieldLineSC::FieldLineSC()
		: Processor(), 
		inportvolspherical_("volumespherical"), 
		inportvolcartesian_("volumecartesian"),

		inportpoint_("PointCloud"),
		outportmesh_("linesStripsMesh_"),
		mesh_inport_("mesh_inport_"),

		outportline_("meshline"),
		//outportlinecartesian_("meshlinecartesian"),

	
		d_("d_", "neighbor distance", 0.01, 0.001, 0.1, 0.001),
		dd_("dd_", "distance", 1.0, 0.1, 1.0, 0.1),
		//m_("m_", "neighbor points", 20, 0, 4, 1),
		numberOfSteps_("steps", "Number of Steps", 100, 1, 1000),
		stepSize_("stepSize", "StepSize", 0.001f, 0.0001f, 1.0f),
		stepDirection_("stepDirection", "Step Direction"),
		integration_type_("integration_type", "Integration Type"),
		tf_("transferFunction_", "Transfer Function"),
		velocityScale_("velocityScale_", "Velocity Scale", 1, 0, 2),
		minMaxVelocity_("minMaxVelocity", "Velocity Range", 0.0f, 1.0f, 0.0f,
		1.0f),
		maxlambda_("maxlambda", "Max Lambda_", 100.0f, 1.0f, 10.0f,
		1.0f)
		, center_("center_", "center_", vec3(0.0f), vec3(0.0f), vec3(0.0f))
		, show_spherical_("show_spherical_", "Show Spherical Coordinate", false)
		//, show_cartesian_("show_spherical_", "show cartesian coordinate", false)

	{
		addPort(inportvolspherical_);
		addPort(inportvolcartesian_);
		addPort(inportpoint_);
		addPort(outportmesh_);
		addPort(mesh_inport_);
		addPort(outportline_);
		//addPort(outportlinecartesian_);

		stepSize_.setIncrement(0.0001f);

		stepDirection_.addOption("fwd", "Forward", FWD);
		stepDirection_.addOption("bwd", "Backwards", BWD);
		stepDirection_.addOption("bi", "Bi Directional", BOTH);


		integration_type_.addOption("euler", "Euler", EULER);
		integration_type_.addOption("rk4", " Runge-Kutta 4th Order", RUNGE_KUTTA_4);


		addProperty(d_);
		addProperty(dd_);
		//addProperty(m_);
		addProperty(maxlambda_);
		addProperty(center_);
		addProperty(show_spherical_);
		//addProperty(show_cartesian_);

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

	FieldLineSC::~FieldLineSC() {}

	vec3 FieldLineSC::euler_cartesian(vec3& p, const VolumeRAM* vector_field, float& stepsize, const int& dir,
		mat3& transformation_matrix, float* velocity) {
		vec3 K1 = trilinear(vector_field, p);
		K1 = glm::normalize(K1) * stepsize;
		vec3 vel = transformation_matrix * K1;
		if (velocity != nullptr) {
			velocity[0] = vel.x;
			velocity[1] = vel.y;
			velocity[2] = vel.z;
		}
		return p + vel * static_cast<float>(dir);
	}

	vec3 FieldLineSC::runge_kutta_4_cartesian(vec3& p, const VolumeRAM*  vector_field, float& stepsize,
		const int& dir, mat3& transformation_matrix, float* velocity) {
		float h = stepsize / 2.f;
		vec3 K1 = trilinear(vector_field, p);
		K1 = glm::normalize(K1) * stepsize;
		//t = (t + h) > 1.f ? (t + h) - 1.f : t + h;
		vec3 new_pos = p + h * (transformation_matrix * K1);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.z = .0; }
		//if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.z = 1.; }
		//if (new_pos.z > 1.) { new_pos.y = (new_pos.y - 1); }
		vec3 K2 = trilinear(vector_field, new_pos);
		K2 = glm::normalize(K2) * stepsize;
		new_pos = p + h * (transformation_matrix * K2);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.z = .0; }
		//if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.z = 1.; }
		//if (new_pos.z > 1.) { new_pos.y = (new_pos.y - 1); }
		vec3 K3 = trilinear(vector_field, new_pos);
		K3 = glm::normalize(K3) * stepsize;
		//t = (t + h) > 1.f ? (t + h) - 1.f : t + h;
		new_pos = p + stepsize * (transformation_matrix * K3);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.z = .0; }
		//if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.z = 1.; }
		//if (new_pos.z > 1.) { new_pos.y = (new_pos.y - 1); }
		vec3 K4 = trilinear(vector_field, new_pos);
		K4 = glm::normalize(K4) * stepsize;
		vec3 vel = (transformation_matrix * K1) * (1.f / 6.f) + (transformation_matrix * K2) * (1.f / 3.f) +
			(transformation_matrix * K3) * (1.f / 3.f) + (transformation_matrix * K4) * (1.f / 6.f);
		if (velocity != nullptr) {
			velocity[0] = vel.x;
			velocity[1] = vel.y;
			velocity[2] = vel.z;
		}
		return p + vel * static_cast<float>(dir);
	}

	vec3 FieldLineSC::euler_spherical(vec3& p, const VolumeRAM* vector_field, float& stepsize, const int& dir,
		mat3& transformation_matrix, float* velocity) {

		vec3 K1 = trilinear(vector_field, p);
		K1 = K1 * stepsize;
		vec3 vel = transformation_matrix * K1;
		if (velocity != nullptr) {
			velocity[0] = vel.x;
			velocity[1] = vel.y;
			velocity[2] = vel.z;
		}
		return p + vel * static_cast<float>(dir);
	}

	vec3 FieldLineSC::runge_kutta_4_spherical(vec3& p, const VolumeRAM*  vector_field, float& stepsize,
		const int& dir, mat3& transformation_matrix, float* velocity) {

		float h = stepsize / 2.f;
		vec3 K1 = trilinear(vector_field, p);
		K1 = K1 * stepsize;
		//t = (t + h) > 1.f ? (t + h) - 1.f : t + h;
		vec3 new_pos = p + h * (transformation_matrix * K1);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		//if (new_pos.z < .0) { new_pos.z = .0; }
		if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		//if (new_pos.z > 1.) { new_pos.z = 1.; }
		if (new_pos.z > 1.) { new_pos.y = (new_pos.y - 1); }
		vec3 K2 = trilinear(vector_field, new_pos);
		K2 = K2 * stepsize;
		new_pos = p + h * (transformation_matrix * K2);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		//if (new_pos.z < .0) { new_pos.z = .0; }
		if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		//if (new_pos.z > 1.) { new_pos.z = 1.; }
		if (new_pos.z > 1.) { new_pos.y = (new_pos.y - 1); }
		vec3 K3 = trilinear(vector_field, new_pos);
		K3 = K3 * stepsize;
		//t = (t + h) > 1.f ? (t + h) - 1.f : t + h;
		new_pos = p + stepsize * (transformation_matrix * K3);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		//if (new_pos.z < .0) { new_pos.z = .0; }
		if (new_pos.z < .0) { new_pos.y = (new_pos.y + 1); }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		//if (new_pos.z > 1.) { new_pos.z = 1.; }
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

	void FieldLineSC::process() {

		if (show_spherical_.get() == true){
			CalculateSpericalLines();
		}
		else{
			CalculateCartesianLines();

		}
	
	}

	void FieldLineSC::CalculateCartesianLines(){

		auto inMesh = mesh_inport_.getData();
		auto meshdata = dynamic_cast<const BasicMesh*>(inMesh.get());
		if (meshdata == nullptr)
			return;
		auto newMesh = meshdata->clone();

		lines_.clear();
		minMaxVelocityTemp_ = vec2(1000000, 0);
		minMaxVelocity_.set(minMaxVelocityTemp_);

		BasicMesh *mesh = new BasicMesh();

		mesh->setModelMatrix(inportvolcartesian_.getData()->getModelMatrix());
		mesh->setWorldMatrix(inportvolcartesian_.getData()->getWorldMatrix());

		auto inputvolume = inportvolcartesian_.getData();

		auto dims = inputvolume->getDimensions();

		auto seedpoints = inportpoint_.getData().get();

		const VolumeRAM *volume =
			inportvolcartesian_.getData()->getRepresentation<VolumeRAM>();

		mat4 index_to_texture =
			inputvolume->getCoordinateTransformer().getIndexToTextureMatrix();
		mat4 model_to_texture =
			inputvolume->getCoordinateTransformer().getModelToTextureMatrix();

		bool fwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(0); // False
		bool bwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(1); // True

		size_t steps = numberOfSteps_.get();
		if (fwd && bwd) { // go both way
			steps /= 2;
		}

		float min = 100000.0f;
		float max = 0.0f;
		size_t idx = 0;
		for (auto it = seedpoints->begin(); it != seedpoints->end(); it++) {

			vec3 pos = *it;
			vec3 N = trilinear(volume, pos);


			//////////////////////////// 4 neighbor points ///////////////////////////////

			vec3 w = glm::normalize(glm::cross(N, vec3(0, 0, 1)));
			if (glm::normalize(N) == w || glm::normalize(N) == -w) {

				w = glm::normalize(glm::cross(N, vec3(0, 1, 0)));
			}

			vec3 a = glm::normalize(glm::cross(N, w));

			auto model_matrix = inMesh->getModelMatrix();
			vec3 p1 = (model_matrix * glm::vec4(*it, 1)).xyz;

			for (size_t i = 0; i < 5; i++) {

				Line l;
				vec3 p;

				if (i == 0) {
					p = p1;
				}
				if (i == 1) {
					p = p1 + d_.get() * w;
				}
				if (i == 2) {
					p = p1 - d_.get() * w;
				}
				if (i == 3) {
					p = p1 + d_.get() * a;
				}
				if (i == 4) {
					p = p1 - d_.get() * a;
				}

				//TODO transform into texture space

				mat4 p_to_texture = inportvolcartesian_.getData()->getCoordinateTransformer().getModelToTextureMatrix();
				p = (p_to_texture * vec4(p, 1)).xyz;

				if (fwd) {
					step_cartesian(volume, mesh, p, steps, 1, &l);
				}
				if (bwd) {
					step_cartesian(volume, mesh, p, steps, -1, &l);
				}

				//l.CalculateLength();
				//LogInfo("Length before trim: " << l.GetLength());
				//if (l.GetLength() > dd_.get()){
				//	//l.Trim(dd_.get());
				//}

				l.CalculateLength();
				LogInfo("Length : " << l.GetLength());

				lines_.push_back(l);

			}

			index_.push_back(idx);
			idx++;
		}

		auto d = index_.begin();
		for (auto it = lines_.begin(); it != lines_.end(); it += 5) {

			float d1 = glm::distance(it->GetPoint(dd_.get()), (it + 1)->GetPoint(dd_.get()));

			float d2 = glm::distance(it->GetPoint(dd_.get()), (it + 2)->GetPoint(dd_.get()));

			float d3 = glm::distance(it->GetPoint(dd_.get()), (it + 3)->GetPoint(dd_.get()));

			float d4 = glm::distance(it->GetPoint(dd_.get()), (it + 4)->GetPoint(dd_.get()));

			/////////////////  compute eigenvalue  ///////////////////

			vec3 e1 = ((it + 1)->GetPoint(dd_.get()) - (it + 2)->GetPoint(dd_.get())) / (d_.get() * 2);
			vec3 e2 = ((it + 3)->GetPoint(dd_.get()) - (it + 4)->GetPoint(dd_.get())) / (d_.get() * 2);

			glm::mat2x3 A = glm::mat2x3(e1[0], e2[0], e1[1], e2[1], e1[2], e2[2]);

			glm::mat3x2 AT = glm::transpose(A);

			auto e = util::glm2eigen(A);

			Eigen::EigenSolver<Eigen::MatrixXf> es(e.transpose()*e);

			LogInfo("The eigenvalues of A are:\n" << es.eigenvalues());

			std::complex<double> lambda1 = (es.eigenvalues()[0]) / maxlambda_.get();
			std::complex<double> lambda2 = (es.eigenvalues()[1]) / maxlambda_.get();

			LogInfo("Consider the first eigenvalue, lambda1 = " << lambda1);
			LogInfo("Consider the second eigenvalue, lambda2 = " << lambda2);

			vec4 c;

			if (lambda1.real() > lambda2.real()){

				c = vec4(lambda1.real(), lambda1.real(), lambda1.real(), 1);

			}
			else{

				c = vec4(lambda2.real(), lambda2.real(), lambda2.real(), 1);

			}

			newMesh->setVertexColor(*d, c);

			float mid = (d1 + d2 + d3 + d4) / 4;

			if (mid > max) {
				max = mid;
			}

			d++;
		}

		for (auto it = lines_.begin(); it != lines_.end(); it += 5) {

			float d1 = glm::distance(it->GetPoint(dd_.get()), (it + 1)->GetPoint(dd_.get()));

			float d2 = glm::distance(it->GetPoint(dd_.get()), (it + 2)->GetPoint(dd_.get()));

			float d3 = glm::distance(it->GetPoint(dd_.get()), (it + 3)->GetPoint(dd_.get()));

			float d4 = glm::distance(it->GetPoint(dd_.get()), (it + 4)->GetPoint(dd_.get()));

			float mid = (d1 + d2 + d3 + d4) / 4;
			mid /= max;

			//color from distance2color
			//vec4 color = distance2color(mid);

			it->CalculateLength();
			float L = it->GetLength();
			vec4 color = distance2color(L);

			(it + 1)->CalculateLength();
			float L1 = (it + 1)->GetLength();
			vec4 color1 = distance2color(L1);

			(it + 2)->CalculateLength();
			float L2 = (it + 2)->GetLength();
			vec4 color2 = distance2color(L2);

			(it + 3)->CalculateLength();
			float L3 = (it + 3)->GetLength();
			vec4 color3 = distance2color(L3);

			(it + 4)->CalculateLength();
			float L4 = (it + 4)->GetLength();
			vec4 color4 = distance2color(L4);

			it->AddToMesh(mesh, color);
			(it + 1)->AddToMesh(mesh, color1);
			(it + 2)->AddToMesh(mesh, color2);
			(it + 3)->AddToMesh(mesh, color3);
			(it + 4)->AddToMesh(mesh, color4);
		}

		newMesh->getEditableRepresentation<MeshRAM>();

		outportmesh_.setData(newMesh);

		outportline_.setData(mesh);

	}

	void FieldLineSC::CalculateSpericalLines(){

		auto inMesh = mesh_inport_.getData();
		auto meshdata = dynamic_cast<const BasicMesh*>(inMesh.get());
		if (meshdata == nullptr)
			return;
		auto newMesh = meshdata->clone();

		//newMesh->setModelMatrix(glm::translate(newMesh->getModelMatrix(), center_.get()));

		lines_.clear();
		minMaxVelocityTemp_ = vec2(1000000, 0);
		minMaxVelocity_.set(minMaxVelocityTemp_);

		BasicMesh *mesh = new BasicMesh();

		//mesh->setModelMatrix(inportvol_.getData()->getModelMatrix());
		mesh->setWorldMatrix(inportvolspherical_.getData()->getWorldMatrix());

		auto inputvolume = inportvolspherical_.getData();

		auto dims = inputvolume->getDimensions();

		auto seedpoints = inportpoint_.getData().get();

		const VolumeRAM *volume =
			inportvolspherical_.getData()->getRepresentation<VolumeRAM>();

		mat4 index_to_texture =
			inputvolume->getCoordinateTransformer().getIndexToTextureMatrix();
		mat4 model_to_texture =
			inputvolume->getCoordinateTransformer().getModelToTextureMatrix();

		bool fwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(0); // False
		bool bwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(1); // True

		size_t steps = numberOfSteps_.get();
		if (fwd && bwd) { // go both way
			steps /= 2;
		}

		float min = 100000.0f;
		float max = 0.0f;
		size_t idx = 0;
		for (auto it = seedpoints->begin(); it != seedpoints->end(); it++) {

			//IndexBufferRAM *index =
			//	mesh->addIndexBuffer(DrawType::LINES, ConnectivityType::STRIP); // Buffer

			vec3 pos = *it;
			vec3 N = trilinear(volume, pos);

			//////////////////////////// 4 neighbor points ///////////////////////////////

			vec3 w = glm::normalize(glm::cross(N, vec3(0, 0, 1)));
			if (glm::normalize(N) == w || glm::normalize(N) == -w) {
				w = glm::normalize(glm::cross(N, vec3(0, 1, 0)));
			}

			vec3 a = glm::normalize(glm::cross(N, w));

			//glm::mat4 model_to_world = glm::scale(glm::mat4(1), glm::vec3(.5));
			//vec3 p1 = (model_to_world * glm::vec4(*it , 1)).xyz;

			auto model_matrix = inMesh->getModelMatrix();
			vec3 p1 = (model_matrix * glm::vec4(*it, 1)).xyz;

			//p1 = (p1 + glm::vec3(1)) / 2.f; //between 0 and 1

			for (size_t i = 0; i < 5; i++) {

				Line l;
				vec3 p;

				if (i == 0) {
					p = p1;
				}
				if (i == 1) {
					p = p1 + d_.get() * w;
				}
				if (i == 2) {
					p = p1 - d_.get() * w;
				}
				if (i == 3) {
					p = p1 + d_.get() * a;
				}
				if (i == 4) {
					p = p1 - d_.get() * a;
				}

				//TODO transform Volume into texture space

				//mat4 p_to_texture = inportvol_.getData()->getCoordinateTransformer().getModelToTextureMatrix();
				//p = (p_to_texture * vec4(p, 1)).xyz;

				p = cartesianToSpherical(p);

				if (fwd) {
					step_spherical(volume, mesh, p, steps, 1, &l);
				}
				if (bwd) {
					step_spherical(volume, mesh, p, steps, -1, &l);
				}

				//l.CalculateLength();
				//LogInfo("Length before trim: " << l.GetLength());
				//if (l.GetLength() > dd_.get()){
				//	//l.Trim(dd_.get());
				//}

				l.CalculateLength();
				LogInfo("Length : " << l.GetLength());

				//L = l.GetLength();

				lines_.push_back(l);

			}

			index_.push_back(idx);
			idx++;
		}

		auto d = index_.begin();
		for (auto it = lines_.begin(); it != lines_.end(); it += 5) {

			float d1 = glm::distance(sphericalToCartesian(it->GetPoint(dd_.get())), sphericalToCartesian((it + 1)->GetPoint(dd_.get())));

			float d2 = glm::distance(sphericalToCartesian(it->GetPoint(dd_.get())), sphericalToCartesian((it + 2)->GetPoint(dd_.get())));

			float d3 = glm::distance(sphericalToCartesian(it->GetPoint(dd_.get())), sphericalToCartesian((it + 3)->GetPoint(dd_.get())));

			float d4 = glm::distance(sphericalToCartesian(it->GetPoint(dd_.get())), sphericalToCartesian((it + 4)->GetPoint(dd_.get())));

			/////////////////  compute eigenvalue  ///////////////////

			vec3 e1 = (sphericalToCartesian((it + 1)->GetPoint(dd_.get())) - sphericalToCartesian((it + 2)->GetPoint(dd_.get()))) / (d_.get() * 2);
			vec3 e2 = (sphericalToCartesian((it + 3)->GetPoint(dd_.get())) - sphericalToCartesian((it + 4)->GetPoint(dd_.get()))) / (d_.get() * 2);

			glm::mat2x3 A = glm::mat2x3(e1[0], e2[0], e1[1], e2[1], e1[2], e2[2]);

			glm::mat3x2 AT = glm::transpose(A);

			auto e = util::glm2eigen(A);

			Eigen::EigenSolver<Eigen::MatrixXf> es(e.transpose()*e);

			LogInfo("The eigenvalues of A are:\n" << es.eigenvalues());

			std::complex<double> lambda1 = (es.eigenvalues()[0]) / maxlambda_.get();
			std::complex<double> lambda2 = (es.eigenvalues()[1]) / maxlambda_.get();

			LogInfo("Consider the first eigenvalue, lambda1 = " << lambda1);
			LogInfo("Consider the second eigenvalue, lambda2 = " << lambda2);

			vec4 c;

			if (lambda1.real() > lambda2.real()){

				c = vec4(lambda1.real(), lambda1.real(), lambda1.real(), 1);

			}
			else{

				c = vec4(lambda2.real(), lambda2.real(), lambda2.real(), 1);

			}

			newMesh->setVertexColor(*d, c);

			float mid = (d1 + d2 + d3 + d4) / 4;

			if (mid > max) {
				max = mid;
			}
            

			d++;
		}

		for (auto it = lines_.begin(); it != lines_.end(); it += 5) {


			float d1 = glm::distance(sphericalToCartesian(it->GetPoint(dd_.get())), sphericalToCartesian((it + 1)->GetPoint(dd_.get())));

			float d2 = glm::distance(sphericalToCartesian(it->GetPoint(dd_.get())), sphericalToCartesian((it + 2)->GetPoint(dd_.get())));

			float d3 = glm::distance(sphericalToCartesian(it->GetPoint(dd_.get())), sphericalToCartesian((it + 3)->GetPoint(dd_.get())));

			float d4 = glm::distance(sphericalToCartesian(it->GetPoint(dd_.get())), sphericalToCartesian((it + 4)->GetPoint(dd_.get())));

			float mid = (d1 + d2 + d3 + d4) / 4;
			mid /= max;

			//color from distance2color
			//vec4 color = distance2color(mid);

			it->CalculateLength();
			float L = it->GetLength();
			vec4 color = distance2color(L);

			(it + 1)->CalculateLength();
			float L1 = (it + 1)->GetLength();
			vec4 color1 = distance2color(L1);

			(it + 2)->CalculateLength();
			float L2 = (it + 2)->GetLength();
			vec4 color2 = distance2color(L2);

			(it + 3)->CalculateLength();
			float L3 = (it + 3)->GetLength();
			vec4 color3 = distance2color(L3);

			(it + 4)->CalculateLength();
			float L4 = (it + 4)->GetLength();
			vec4 color4 = distance2color(L4);

			it->AddToMeshSphericalToCartesian(mesh, color);
			(it + 1)->AddToMeshSphericalToCartesian(mesh, color1);
			(it + 2)->AddToMeshSphericalToCartesian(mesh, color2);
			(it + 3)->AddToMeshSphericalToCartesian(mesh, color3);
			(it + 4)->AddToMeshSphericalToCartesian(mesh, color4);

		}

		newMesh->getEditableRepresentation<MeshRAM>();

		outportmesh_.setData(newMesh);

		outportline_.setData(mesh);



	}
	void FieldLineSC::step_cartesian(const VolumeRAM *volume, BasicMesh *mesh,
		const vec3 &startPos, const size_t &steps, const int &dir, Line *l) {
		vec3 currentPos = startPos;
		l->AddPoint(startPos);

		IndexBufferRAM *idx_buffer =
				mesh->addIndexBuffer(DrawType::LINES, ConnectivityType::STRIP); // Buffer

		auto dim = volume->getDimensions();
		float lengt = 0;
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


		/*	if (currentPos.x < 0)
				break;
			if (currentPos.y < 0)
				break;
			if (currentPos.z < 0)
				currentPos.z = ((currentPos.z) + 1);

			if (currentPos.x > 1)
				break;
			if (currentPos.y > 1)
				break;
			if (currentPos.z > 1)
				currentPos.z = ((currentPos.z) - 1);*/

			/*if (!util::CheckBoundsForPoint(currentPos, 0.0, 1.0))
				break;*/

			if (currentPos.x > 1 - 1.0 / dim.x)
				break;
			if (currentPos.y > 1 - 1.0 / dim.y)
				break;
			if (currentPos.z > 1 - 1.0 / dim.z)
				break;

			float *vel = new float[3];

			vec3 perPos;

			auto m = glm::inverse(inportvolcartesian_.getData()->getBasis());

			if (integration_type_.get() == EULER) { //TODO change mat3 to inv basis from volume (to convert to texture space=
				perPos = euler_cartesian(currentPos, volume, stepSize_.get(), dir, m, vel);
			}

			if (integration_type_.get() == RUNGE_KUTTA_4) {//TODO change mat3 to inv basis from volume (to convert to texture space=
				perPos = runge_kutta_4_cartesian(currentPos, volume, stepSize_.get(), dir, m, vel);
			}

			if (vel == 0){ //vel terminate

				break;
			}


			l->AddPoint(perPos);

			currentPos = perPos;


		}
	}

	void FieldLineSC::step_spherical(const VolumeRAM *volume, BasicMesh *mesh,
		const vec3 &startPos, const size_t &steps, const int &dir, Line *l) {

		vec3 currentPos = startPos;
		l->AddPoint(startPos);

		IndexBufferRAM *idx_buffer =
			mesh->addIndexBuffer(DrawType::LINES, ConnectivityType::STRIP); // Buffer

		auto dim = volume->getDimensions();
		//float lengt = 0;
		for (size_t i = 0; i < steps; i++) {

			if (currentPos.x < 0)
				break;
			if (currentPos.y < 0)
				break;
			if (currentPos.z < 0)
				currentPos.z = ((currentPos.z) + 1);

			if (currentPos.x > 1)
				break;
			if (currentPos.y > 1)
				break;
			if (currentPos.z > 1)
				currentPos.z = ((currentPos.z) - 1);

			/*if (currentPos.x > 1 - 1.0 / dim.x)
			break;
			if (currentPos.y > 1 - 1.0 / dim.y)
			break;
			if (currentPos.z > 1 - 1.0 / dim.z)
			break;*/

			float *vel = new float[3];

			vec3 perPos;

			auto m = glm::inverse(inportvolspherical_.getData()->getBasis()); //TODO change mat3 to inv basis from volume (to convert to texture space=

			if (integration_type_.get() == EULER) {
				perPos = euler_spherical(currentPos, volume, stepSize_.get(), dir, m, vel);

			}


			if (integration_type_.get() == RUNGE_KUTTA_4) {
				perPos = runge_kutta_4_spherical(currentPos, volume, stepSize_.get(), dir, m, vel);
			}
			if (vel == 0){ //vel terminate

				break;
			}

			l->AddPoint(perPos);

			currentPos = perPos;


		}
	}

	vec4 FieldLineSC::velocity2color(const vec3 &veloicty) {

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

	vec4 FieldLineSC::distance2color(float idx) {

		const LayerRAM *tf = tf_.get().getData()->getRepresentation<LayerRAM>();

		if (idx > 1)
			idx = 1;
		if (idx < 0)
			idx = 0;

		uvec2 pos(idx * (tf->getDimensions().x - 1), 0);

		return (vec4)tf->getValueAsVec4Double(pos);
	}

	

} // namespace
