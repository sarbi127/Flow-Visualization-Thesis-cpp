

#include "FieldLine.h"

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
	ProcessorCodeState(FieldLine, CodeState::Experimental);

	FieldLine::FieldLine()
		: Processor(), 
		inportvol_("vectorvolume"), 
		inportpoint_("PointCloud"),
		outportmesh_("linesStripsMesh_"),
		mesh_inport_("mesh_inport_"),
		outportline_("meshline"),

	
		d_("d_", "neighbor distance", 0.01, 0.001, 0.1, 0.001),
		dd_("dd_", "distance", 1.0, 0.1, 1.0, 0.1),
		m_("m_", "neighbor points", 20, 0, 4, 1),
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


		addProperty(d_);
		addProperty(dd_);
		addProperty(m_);
		addProperty(maxlambda_);
		addProperty(center_);

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
		K1 = glm::normalize(K1) * stepsize;
		vec3 vel = transformation_matrix * K1;
		if (velocity != nullptr) {
			velocity[0] = vel.x;
			velocity[1] = vel.y;
			velocity[2] = vel.z;
		}
		return p + vel * static_cast<float>(dir);
	}

	vec3 FieldLine::runge_kutta_4(vec3& p, const VolumeRAM*  vector_field, float& stepsize,
		const int& dir, mat3& transformation_matrix, float* velocity) {
		float h = stepsize / 2.f;
		vec3 K1 = trilinear(vector_field, p);
		K1 = glm::normalize(K1) * stepsize;
		//t = (t + h) > 1.f ? (t + h) - 1.f : t + h;
		vec3 new_pos = p + h * (transformation_matrix * K1);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.z = .0; }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.z = 1.; }
		vec3 K2 = trilinear(vector_field, new_pos);
		K2 = glm::normalize(K2) * stepsize;
		new_pos = p + h * (transformation_matrix * K2);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.z = .0; }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.z = 1.; }
		vec3 K3 = trilinear(vector_field, new_pos);
		K3 = glm::normalize(K3) * stepsize;
		//t = (t + h) > 1.f ? (t + h) - 1.f : t + h;
		new_pos = p + stepsize * (transformation_matrix * K3);
		if (new_pos.x < .0) { new_pos.x = .0; }
		if (new_pos.y < .0) { new_pos.y = .0; }
		if (new_pos.z < .0) { new_pos.z = .0; }
		if (new_pos.x > 1.) { new_pos.x = 1.; }
		if (new_pos.y > 1.) { new_pos.y = 1.; }
		if (new_pos.z > 1.) { new_pos.z = 1.; }
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

	void FieldLine::process() {

		/*if (show_cartesian_.get()) {
			CalculateCartesianLines();
		}
		if (show_spherical_.get()){
			CalculateSpericalLines();
		}
	}*/
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

			/////////////////////////////////////// GetNeighbour /////////////////////////////////////////

			/*std::vector<vec3> Npoints = GetNeighbor(N, *it, m_.get());
			 Npoints.push_back(*it);

			 for (size_t i = 0; i < 2; i++) {

			 vec3 p;

			 if (i = 0){

			 Line l1;
			 p = *it;

			 if (fwd) {

			 step(volume, mesh, p, steps, 1, &l1);

			 }
			 if (bwd) {

			 step(volume, mesh, p, steps, -1, &l1);

			 }

			 l1.CalculateLength();
			 LogInfo("Length before trim: " << l1.GetLength());
			 if (l1.GetLength() > dd_.get()){
			 l1.Trim(dd_.get());
			 }

			 l1.CalculateLength();
			 LogInfo("Length after trim: " << l1.GetLength());

			 lines_.push_back(l1);

			 }

			 if (i = 1){

			 for (auto p : Npoints){

			 Line l2;

			 if (fwd) {

			 step(volume, mesh, p, steps, 1, &l2);

			 }
			 if (bwd) {

			 step(volume, mesh, p, steps, -1, &l2);

			 }

			 l2.CalculateLength();
			 LogInfo("Length before trim: " << l2.GetLength());
			 if (l2.GetLength() > dd_.get()){
			 l2.Trim(dd_.get());
			 }

			 l2.CalculateLength();
			 LogInfo("Length after trim: " << l2.GetLength());

			 lines_.push_back(l2);

			 }

			 }

			 }*/

			//size3_t currentPosIndexSpace = size3_t(*it * vec3(inportvol_.getData()->getDimensions()));//textur coor to Index coor(0,1)

			//p.p1 = (currentPosIndexSpace + vec3(1, 0, 0)) / vec3(inportvol_.getData()->getDimensions());//divide it to back to textur coor

			//////////////////////////// 4 neighbor points ///////////////////////////////

			//neighbor p;

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

				//TODO transform into texture space

				mat4 p_to_texture = inportvol_.getData()->getCoordinateTransformer().getModelToTextureMatrix();
				p = (p_to_texture * vec4(p, 1)).xyz;

				//mat4 p_to_texture = mat4 (inMesh->getBasis());
				//p = (p_to_texture * glm::vec4(*it, 1)).xyz;


				//mat3 p_to_texture = (inMesh->getBasis());
				//auto offset = inMesh->getOffset();
			    //p = (p_to_texture * p);

				

				if (fwd) {
					step(volume, mesh, p, steps, 1, &l);
				}
				if (bwd) {
					step(volume, mesh, p, steps, -1, &l);
				}

				l.CalculateLength();
				LogInfo("Length before trim: " << l.GetLength());
				if (l.GetLength() > dd_.get()){
					//l.Trim(dd_.get());
				}

				l.CalculateLength();
				LogInfo("Length after trim: " << l.GetLength());

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

				vec3 e1 = ((it + 1)->GetPoint(dd_.get()) - (it + 2)->GetPoint(dd_.get())) /( d_.get() * 2);
				vec3 e2 = ((it + 3)->GetPoint(dd_.get()) - (it + 4)->GetPoint(dd_.get())) / (d_.get() * 2);

				glm::mat2x3 A = glm::mat2x3(e1[0], e2[0], e1[1], e2[1], e1[2], e2[2]);
				//glm::mat3x2 A = glm::mat3x2(e1[0], e1[1], e1[2], e2[0], e2[1], e2[2]);

				glm::mat3x2 AT = glm::transpose(A);

				//glm::mat3 M = AT * A;

				auto e = util::glm2eigen(A);

				Eigen::EigenSolver<Eigen::MatrixXf> es(e.transpose()*e); 

			   LogInfo("The eigenvalues of A are:\n" << es.eigenvalues());

			   std::complex<double> lambda1 = (es.eigenvalues()[0]) / maxlambda_.get();
			   std::complex<double> lambda2 = (es.eigenvalues()[1]) / maxlambda_.get();

				LogInfo("Consider the first eigenvalue, lambda1 = "  << lambda1);
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
				vec4 color = distance2color(mid);
				//vec4 color(.5);
				//for it -> it+4 call AddToMesh
				
				it->AddToMesh(mesh, color);
				(it + 1)->AddToMesh(mesh, color);
				(it + 2)->AddToMesh(mesh, color);
				(it + 3)->AddToMesh(mesh, color);
				(it + 4)->AddToMesh(mesh, color);
			}

			newMesh->getEditableRepresentation<MeshRAM>();
			
			outportmesh_.setData(newMesh);

			outportline_.setData(mesh);

	}
	void FieldLine::step(const VolumeRAM *volume, BasicMesh *mesh,
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

			/*if (!util::CheckBoundsForPoint(currentPos, 0.0, 1.0))
				break;*/

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

			auto m = glm::inverse(inportvol_.getData()->getBasis());

			if (integration_type_.get() == EULER) { //TODO change mat3 to inv basis from volume (to convert to texture space=
				perPos = euler(currentPos, volume, stepSize_.get(), dir, m, vel);
			}

			if (integration_type_.get() == RUNGE_KUTTA_4) {//TODO change mat3 to inv basis from volume (to convert to texture space=
				perPos = runge_kutta_4(currentPos, volume, stepSize_.get(), dir, m, vel);
			}


			l->AddPoint(perPos);

			//vec3 velocity = vec3(vel[0], vel[1], vel[2]);

			//delete vel;


			////*************************************

			//vec4 color = velocity2color(velocity);

			//size_t idx1_ = mesh->addVertex(currentPos, currentPos, vec3(1), vec4(1.0f, 0.0f, 0.0f, 1.0f));

			/*size_t idx1_ = mesh->addVertex(perPos, perPos, vec3(1), color);

			size_t idx2_ = mesh->addVertex(currentPos, currentPos, vec3(1), color);*/

			//////currentPos = currentPos + velocity.xyz() * stepSize_.get() * static_cast<float>(dir);

			//idx_buffer->add(static_cast<uint32_t>(idx1_)); // Buffer
			//idx_buffer->add(static_cast<uint32_t>(idx2_)); // Buffer

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

	vec4 FieldLine::distance2color(float idx) {

		const LayerRAM *tf = tf_.get().getData()->getRepresentation<LayerRAM>();

		if (idx > 1)
			idx = 1;
		if (idx < 0)
			idx = 0;

		uvec2 pos(idx * (tf->getDimensions().x - 1), 0);

		return (vec4)tf->getValueAsVec4Double(pos);
	}

	/*std::vector<vec3> FieldLine::GetNeighbor(vec3 n, vec3 seedpoint, int m) {

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
	}*/

	/*float FieldLine::eigenvalues(vec3 p0, vec3 p1, vec3 p2, vec3 p3){

		float Landa;

		 
	    glm::mat2  A = mat2(p0 - p1, p2 - p3);

		glm::mat2 I = mat2(1, 0);

	   	A -( I * Landa) = 0;

	}*/

	/*float FieldLine::CalculateDistance(vec3& p1, const std::vector<vec3>& p2)
	{
	float deltaX;
	for (auto p : p2) {
	deltaX = glm::distance(p1, p);//square
	}

	return deltaX;
	}*/

} // namespace
