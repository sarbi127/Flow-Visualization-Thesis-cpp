
#include "FieldLineCartesianCo.h"

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

#define FTLE 1
#define ANGLE 2

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

	ProcessorClassIdentifier(FieldLineCartesianCo, "org.inviwo.FieldLineCartesianCo")
	ProcessorDisplayName(FieldLineCartesianCo, "FieldLineCartesianCo")
	ProcessorTags(FieldLineCartesianCo, Tags::None);
	ProcessorCategory(FieldLineCartesianCo, "Vector Field Visualization");
	ProcessorCodeState(FieldLineCartesianCo, CodeState::Experimental);

	FieldLineCartesianCo::FieldLineCartesianCo()
		: Processor(),
		inportvol_("vectorvolume"),
		inportpoint_("PointCloud"),
		outportmesh_("linesStripsMesh_"),
		mesh_inport_("mesh_inport_"),
		outportline_("meshline"),


		d_("d_", "neighbor distance", 0.01, 0.001, 0.1, 0.001),
		dd_("dd_", "distance", 1.0, 0.1, 1.0, 0.1),
		//m_("m_", "neighbor points", 20, 0, 4, 1),
		numberOfSteps_("steps", "Number of Steps", 100, 1, 1000),
		stepSize_("stepSize", "StepSize", 0.001f, 0.0001f, 1.0f),
		stepDirection_("stepDirection", "Step Direction"),
		integration_type_("integration_type", "Integration Type"),
		color_("color", "Vertex Color Type"),
		tf_("transferFunction_", "Transfer Function"),
		velocityScale_("velocityScale_", "Velocity Scale", 1, 0, 2),
		minMaxVelocity_("minMaxVelocity", "Velocity Range", 0.0f, 1.0f, 0.0f,
		1.0f),
		maxlambda_("maxlambda", "Max Lambda_", 100.0f, 1.0f, 10.0f,
		1.0f),
		center_("center_", "center_", vec3(0.0f), vec3(0.0f), vec3(0.0f))
		//spherical_pro_("spherical_pro_", "Show Spherical Coordinate", false)


	{
		addPort(inportvol_);
		addPort(inportpoint_);
		addPort(outportmesh_);
		addPort(mesh_inport_);
		addPort(outportline_);

		stepSize_.setIncrement(0.0001f);

		stepDirection_.addOption("fwd", "Forward", FWD);
		stepDirection_.addOption("bwd", "Backwards", BWD);
		stepDirection_.addOption("bi", "Bi Directional", BOTH);


		integration_type_.addOption("euler", "Euler", EULER);
		integration_type_.addOption("rk4", " Runge-Kutta 4th Order", RUNGE_KUTTA_4);

		color_.addOption("ftle", "FTLE", FTLE);
		color_.addOption("angle", "ANGLE", ANGLE);

		stepSize_.setCurrentStateAsDefault();
		stepDirection_.setCurrentStateAsDefault();

		minMaxVelocity_.setReadOnly(true);

		addProperty(d_);
		addProperty(dd_);
		//addProperty(m_);
		addProperty(maxlambda_);
		addProperty(center_);
		//addProperty(spherical_pro_);
		addProperty(numberOfSteps_);
		addProperty(stepSize_);
		addProperty(stepDirection_);
		addProperty(integration_type_);
	    addProperty(color_);
		addProperty(tf_);
		addProperty(velocityScale_);
		addProperty(minMaxVelocity_);

		tf_.get().clearPoints();
		tf_.get().addPoint(vec2(0, 1), vec4(0, 0, 1, 1));
		tf_.get().addPoint(vec2(0.5, 1), vec4(1, 1, 0, 1));
		tf_.get().addPoint(vec2(1, 1), vec4(1, 0, 0, 1));

		tf_.setCurrentStateAsDefault();
	}

	FieldLineCartesianCo::~FieldLineCartesianCo() {}

	vec3 FieldLineCartesianCo::euler(vec3& p, const VolumeRAM* vector_field, float& stepsize, const int& dir,
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

	vec3 FieldLineCartesianCo::runge_kutta_4(vec3& p, const VolumeRAM*  vector_field, float& stepsize,
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

	void FieldLineCartesianCo::process() {

		/*auto inMesh = mesh_inport_.getData();
		auto meshdata = dynamic_cast<const BasicMesh*>(inMesh.get());
		if (meshdata == nullptr)
			return;
		auto newMesh = meshdata->clone();*/

		//newMesh->setModelMatrix(glm::translate(newMesh->getModelMatrix(), center_.get()));

		lines_.clear();

		minMaxVelocityTemp_ = vec2(1000000, 0);
		minMaxVelocity_.set(minMaxVelocityTemp_);

		BasicMesh *mesh = new BasicMesh();

		//CheckBox
		/*if (spherical_pro_){
			mesh->setWorldMatrix(inportvol_.getData()->getWorldMatrix());
		}
		else{
			mesh->setModelMatrix(inportvol_.getData()->getModelMatrix());
			mesh->setWorldMatrix(inportvol_.getData()->getWorldMatrix());
		}*/

		mesh->setModelMatrix(inportvol_.getData()->getModelMatrix());
		mesh->setWorldMatrix(inportvol_.getData()->getWorldMatrix());

		auto inputvolume = inportvol_.getData();
		auto dims = inputvolume->getDimensions();

		auto seedpoints = inportpoint_.getData().get();

		const VolumeRAM *volume =
			inportvol_.getData()->getRepresentation<VolumeRAM>();

		mat4 index_to_texture =
			inputvolume->getCoordinateTransformer().getIndexToTextureMatrix();
		mat4 model_to_texture =
			inputvolume->getCoordinateTransformer().getModelToTextureMatrix();
		
		Calculate4Neighbor(*seedpoints, volume, mesh);
		CalculateVertexColor(volume, mesh, *seedpoints);
		AddColorLinesToMesh( mesh);

	}

	void FieldLineCartesianCo::Calculate4Neighbor(std::vector<vec3> points, const VolumeRAM *volume, BasicMesh *mesh ){

		bool fwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(0); // False
		bool bwd = std::bitset<sizeof(int)>(stepDirection_.get()).test(1); // True

		index_.clear();

		size_t steps = numberOfSteps_.get();
		if (fwd && bwd) { // go both way
			steps /= 2;
		}

		size_t idx = 0;
		for (auto it = points.begin(); it != points.end(); it++) {

			//IndexBufferRAM *index =
			//	mesh->addIndexBuffer(DrawType::LINES, ConnectivityType::STRIP); // Buffer

			vec3 pos = *it;

			/*vec3 posspherical = cartesianToSpherical(pos);

			vec3 N = trilinear(volume, posspherical);*/

			vec3 N = trilinear(volume, pos);

			//N = sphericalToCartesian(N);

		    /*if (glm::length(N) < 0.000001f){

				idx++;
				continue;
			}*/

			vec3 w = glm::normalize(glm::cross(N, vec3(0, 0, 1)));

			if (std::abs(glm::dot(N, vec3(0, 0, 1))) == 1) {
				w = glm::normalize(glm::cross(N, vec3(0, 1, 0)));
			}

			vec3 a = glm::normalize(glm::cross(N, w));

			//glm::mat4 model_to_world = glm::scale(glm::mat4(1), glm::vec3(.5));
			//vec3 p1 = (model_to_world * glm::vec4(*it , 1)).xyz;

			auto model_matrix = mesh_inport_.getData()->getModelMatrix();
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
				mat4 p_to_texture = inportvol_.getData()->getCoordinateTransformer().getModelToTextureMatrix();
				p = (p_to_texture * vec4(p, 1)).xyz;

				//p = cartesianToSpherical(p);

				if (fwd) {
					step(volume, mesh, p, steps, 1, &l);
				}
				if (bwd) {
					step(volume, mesh, p, steps, -1, &l);
				}

				l.CalculateLength();
				//LogInfo("Length before trim: " << l.GetLength());
				if (l.GetLength() > dd_.get()){
					//l.Trim(dd_.get());
				}

				l.CalculateLength();
				//LogInfo("Length after trim: " << l.GetLength());

				lines_.push_back(l);

			}

			index_.push_back(idx);
			idx++;
			//break;
		}

	}

	float FieldLineCartesianCo::radianToDegree(float rad) {
		return rad * (180.f / M_PI);
	}

	void FieldLineCartesianCo::CalculateVertexColor(const VolumeRAM *volume, BasicMesh *mesh, std::vector<vec3> points){

		float max1 = 0.0f;

		auto inMesh = mesh_inport_.getData();
		auto meshdata = dynamic_cast<const BasicMesh*>(inMesh.get());
		if (meshdata == nullptr)
			return;
		auto newMesh = meshdata->clone();

		auto IndexSeedPoint = index_.begin();

		float maxlbda = 0.0f;
		size_t counter = 0;
		for (auto it = lines_.begin(); it != lines_.end(); it += 5) {

			point_.push_back(it->GetPoints()->at(0));

			float d1 = glm::distance((it->GetPoint(dd_.get())), ((it + 1)->GetPoint(dd_.get())));

			float d2 = glm::distance((it->GetPoint(dd_.get())), ((it + 2)->GetPoint(dd_.get())));

			float d3 = glm::distance((it->GetPoint(dd_.get())), ((it + 3)->GetPoint(dd_.get())));

			float d4 = glm::distance((it->GetPoint(dd_.get())), ((it + 4)->GetPoint(dd_.get())));

			/////////////////  compute eigenvalue  ///////////////////

			vec3 e1 = (((it + 1)->GetPoint(dd_.get())) - ((it + 2)->GetPoint(dd_.get()))) / (d_.get() * 2);
			vec3 e2 = (((it + 3)->GetPoint(dd_.get())) - ((it + 4)->GetPoint(dd_.get()))) / (d_.get() * 2);

			/*vec3 e1 = (((it + 1)->GetPoint(dd_.get())) - ((it + 2)->GetPoint(dd_.get())));
			vec3 e2 = (((it + 3)->GetPoint(dd_.get())) - ((it + 4)->GetPoint(dd_.get())));*/

			glm::mat2x3 A = glm::mat2x3(e1[0], e2[0], e1[1], e2[1], e1[2], e2[2]);

			glm::mat3x2 AT = glm::transpose(A);

			auto e = util::glm2eigen(A);

			Eigen::EigenSolver<Eigen::MatrixXf> es(e.transpose()*e);

			//LogInfo("The eigenvalues of A are:\n" << es.eigenvalues());

			std::complex<double> lambda1 = (es.eigenvalues()[0]);
			std::complex<double> lambda2 = (es.eigenvalues()[1]);

			//LogInfo("Consider the first eigenvalue, lambda1 = " << lambda1);
			//LogInfo("Consider the second eigenvalue, lambda2 = " << lambda2);

			vec4 color;

			//vec3 p = it->GetPoints()->at(0);

			//p = glm::normalize(sphericalToCartesian(p));
			//p = cartesianToSpherical(p);
			//color.r = glm::clamp(p.x, 0.f, 1.f);


			//color.r = p.x;
			//color.g = (p.y / 0.3f) / (M_PI); //theta 0° to  180°
			//color.b = (p.y / 0.3f) / (M_PI); //phi 0° to  360°
			//color.a = (p.y / 0.3f) / (M_PI);



			if (color_.get() == FTLE) {

				if (lambda1.real() > maxlbda){
					maxlbda = lambda1.real();
				}

				if (lambda2.real() > maxlbda){
					maxlbda = lambda2.real();
				}

				if (lambda1.real() > lambda2.real()){
					color = vec4(lambda1.real() / maxlambda_.get(), lambda1.real() / maxlambda_.get(), lambda1.real() / maxlambda_.get(), 1);
				}
				else{
					color = vec4(lambda2.real() / maxlambda_.get(), lambda2.real() / maxlambda_.get(), lambda2.real() / maxlambda_.get(), 1);
				}

			}

			/*if (point_.size() == 3){

				vec3 p1 = point_.back();
				point_.pop_back();

				vec3 p2 = point_.back();
				point_.pop_back();

				vec3 p3 = point_.back();
				point_.pop_back();

				face_.push_back(face(p1, p2, p3));

				vec3 Np1 = trilinear(volume, p1);
				vec3 Np2 = trilinear(volume, p2);
				vec3 Np3 = trilinear(volume, p3);

				auto dotNp1 = glm::dot(Np1, vec3(0, 0, 1));
				auto dotNp2 = glm::dot(Np2, vec3(0, 0, 1));
				auto dotNp3 = glm::dot(Np3, vec3(0, 0, 1));

				if ((dotNp1 == 1 && dotNp2 == -1 && dotNp3 == -1)){

				vec3 p12 = linear(p1, p2, 1);
				vec3 p13 = linear(p1, p3, 1);

				points.push_back(p12);
				points.push_back(p13);

				line_.push_back(line(p12, p13));

				}

				if ((dotNp2 == 1 && dotNp1 == -1 && dotNp3 == -1)){

				vec3 p21 = linear(p2, p1, 1);
				vec3 p23 = linear(p2, p3, 1);

				points.push_back(p21);
				points.push_back(p23);

				line_.push_back(line(p21, p23));

				}

				if ((dotNp3 == 1 && dotNp1 == -1 && dotNp2 == -1)){

				vec3 p31 = linear(p3, p1, 1);
				vec3 p32 = linear(p3, p2, 1);

				points.push_back(p31);
				points.push_back(p32);

				line_.push_back(line(p31, p32));

				}

				if ((dotNp1 == 1 && dotNp2 == 1 && dotNp3 == -1)){

				vec3 p13 = linear(p1, p3, 1);
				vec3 p23 = linear(p2, p3, 1);

				points.push_back(p13);
				points.push_back(p23);

				line_.push_back(line(p13, p23));

				}

				if ((dotNp1 == 1 && dotNp2 == -1 && dotNp3 == 1)){

				vec3 p12 = linear(p1, p2, 1);
				vec3 p32 = linear(p3, p2, 1);

				points.push_back(p12);
				points.push_back(p32);

				line_.push_back(line(p12, p32));

				}

				if ((dotNp1 == -1 && dotNp2 == 1 && dotNp3 == 1)){

				vec3 p12 = linear(p1, p2, 1);
				vec3 p13 = linear(p1, p3, 1);

				points.push_back(p12);
				points.push_back(p13);

				line_.push_back(line(p12, p13));

				}

				point_.clear();
				}*/

			//if (color_.get() == ANGLE) {
			//	auto original_position = (it->GetPoints()->at(0) * 2.f) - 1.f;
			//	auto posnor = glm::normalize(original_position);

			//	//auto posnor = it->GetPoints()->at(0);
			//	//vec3 N = trilinear(volume, posnor);

			//	auto vec = glm::normalize(it->GetPoints()->at(1) - (it->GetPoints()->at(0)));

			//	auto ang = radianToDegree(glm::acos(glm::dot(posnor, vec)));
			//	LogInfo("Angle between " << posnor << " and " << vec << " is " << ang);

			//	if (ang == 180.f || ang == 0.f) { //ang=0
			//		color = vec4(1, 0, 0, 1);
			//	}
			//	else if (ang > 90.f) { //inside
			//		color = vec4(0, 1, 0, 1); //green	
			//	}
			//	else { //ang < 90.f outside
			//		color = vec4(0, 0, 1, 1); //blue
			//	}
			//	
			//	counter++;
			//}


			//if (color_.get() == ANGLE) {

			//	auto original_position = it->GetPoints()->at(0);
			//	auto posnor = glm::normalize(original_position);

			//	//posnor = sphericalToCartesian(posnor);

			//	auto vec = glm::normalize(it->GetPoints()->at(1) - (it->GetPoints()->at(0)));

			//	auto ang = glm::dot(posnor, vec);

			//	LogInfo("angle = " << ang);

			//	if (ang > 0.f) {
			//		color = vec4(0, 1+ang, 0, 1); //green
			//	}
			//	else {
			//		color = vec4(0, 0, 1-ang, 1); //blue
			//	}


			//}

			newMesh->setVertexColor(*IndexSeedPoint, color);

			float mid = (d1 + d2 + d3 + d4) / 4;

			if (mid > max1) {
				max1 = mid;
			}

			IndexSeedPoint++;
		}
		LogInfo("Number of angles calculated: " << counter);
		//LogInfo(" maximumlambda = " << maxlbda);

		newMesh->getEditableRepresentation<MeshRAM>();

		outportmesh_.setData(newMesh);
		outportline_.setData(mesh);

	}


	void FieldLineCartesianCo::AddColorLinesToMesh(BasicMesh *mesh){

		float max = 0.0f;

		for (auto it = lines_.begin(); it != lines_.end(); it += 5) {

			float d1 = glm::distance((it->GetPoint(dd_.get())), ((it + 1)->GetPoint(dd_.get())));

			float d2 = glm::distance((it->GetPoint(dd_.get())), ((it + 2)->GetPoint(dd_.get())));

			float d3 = glm::distance((it->GetPoint(dd_.get())), ((it + 3)->GetPoint(dd_.get())));

			float d4 = glm::distance((it->GetPoint(dd_.get())), ((it + 4)->GetPoint(dd_.get())));

			float mid = (d1 + d2 + d3 + d4) / 4;
			mid /= max;

			it->CalculateLength();
			float L = it->GetLength();
			vec4 color = distance2color(L);

			/*(it + 1)->CalculateLength();
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
			vec4 color4 = distance2color(L4);*/

			//vec4 color = distance2color(mid);

			//CheckBox
			/*if (spherical_pro_){

				it->AddToMesh(mesh, color);
				(it + 1)->AddToMesh(mesh, color1);
				(it + 2)->AddToMesh(mesh, color2);
				(it + 3)->AddToMesh(mesh, color3);
				(it + 4)->AddToMesh(mesh, color4);
			}
			else{*/

				it->AddToMesh(mesh, color);
				/*(it + 1)->AddToMesh(mesh, color1);
				(it + 2)->AddToMesh(mesh, color2);
				(it + 3)->AddToMesh(mesh, color3);
				(it + 4)->AddToMesh(mesh, color4);*/
			//}

		}

	}

	void FieldLineCartesianCo::step(const VolumeRAM *volume, BasicMesh *mesh,
		const vec3 &startPos, const size_t &steps, const int &dir, Line *l) {

		vec3 currentPos = startPos;
		l->AddPoint(startPos);

		IndexBufferRAM *idx_buffer =
			mesh->addIndexBuffer(DrawType::LINES, ConnectivityType::STRIP); // Buffer

		int i = 0;
		auto dim = volume->getDimensions();

	    for (size_t i = 0; i < steps; i++) {

		//while (true){


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

			/*if (currentPos.x < 0)
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

			if (currentPos.x > 1 - 1.0 / dim.x)
				break;
				if (currentPos.y > 1 - 1.0 / dim.y)
				break;
				if (currentPos.z > 1 - 1.0 / dim.z)
				break;

			vec3 vel;
			//float *vel = new float[3];
			vec3 perPos;

			auto m = glm::inverse(inportvol_.getData()->getBasis()); //TODO change mat3 to inv basis from volume (to convert to texture space=

			if (integration_type_.get() == EULER) {
				perPos = euler(currentPos, volume, stepSize_.get(), dir, m, glm::value_ptr(vel));

			}

			if (integration_type_.get() == RUNGE_KUTTA_4) {
				perPos = runge_kutta_4(currentPos, volume, stepSize_.get(), dir, m, glm::value_ptr(vel));

			}


			if (vel == vec3(0.0f)){ //vel terminate
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

	vec4 FieldLineCartesianCo::velocity2color(const vec3 &veloicty) {

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

	vec4 FieldLineCartesianCo::distance2color(float idx) {

		const LayerRAM *tf = tf_.get().getData()->getRepresentation<LayerRAM>();

		if (idx > 1)
			idx = 1;
		if (idx < 0)
			idx = 0;

		uvec2 pos(idx * (tf->getDimensions().x - 1), 0);

		return (vec4)tf->getValueAsVec4Double(pos);
	}

} // namespace
