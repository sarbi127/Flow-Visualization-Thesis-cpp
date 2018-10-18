

#include "Disc.h"
#include <inviwo/core/common/inviwo.h>
#include <bitset>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/opengl/image/layergl.h>
#include <inviwo/core/datastructures/image/layerram.h>

//#define PI 3.14

namespace inviwo {

ProcessorClassIdentifier(Disc, "org.inviwo.Disc")
ProcessorDisplayName(Disc, "Disc")
ProcessorTags(Disc, Tags::None);
ProcessorCategory(Disc, "Disc Visualization");
ProcessorCodeState(Disc, CodeState::Experimental);

Disc::Disc()
	: Processor(),
	 outportmesh_("outportmesh_")
	, subdivide_btn_("subdivide_btn", "Subdivide")
	, subdividebackward_btn_("subdividebackward_btn", "Subdivide backward")
	, outportpoint_("vec3_outport")
	, radius_("sphereRadius", "Radius", 0.5f, 0.01f, 0.5f, .01f)
	, vertices_(new std::vector<vec3>())
	, outportface_("face_outport")
	, center_("center_", "center_", vec3(0.0f), vec3(0.0f), vec3(0.0f))
	, faces_(new std::vector<Face>())
	
{

	addPort(outportmesh_);
	addPort(outportpoint_);
	addPort(outportface_);

	addProperty(subdivide_btn_);
	addProperty(subdividebackward_btn_);
	addProperty(radius_);
    addProperty(center_);

	subdivide_btn_.onChange([&]() { onButtonPress(); });

	subdividebackward_btn_.onChange([&]() { onButtonPressback(); });

	model_matrix_ = glm::mat4(1);
	
	//double theta = 26.56505117707799 * PI / 180.0; // refer paper for theta value
	//double stheta = 1;

	double ctheta = 1;

	vertices_->push_back(vec3(0.0f, 0.0f, 0.0f)); // the lower vertex

	// the lower pentagon
	double phi = M_PI / 5.0;
	for (int i = 1; i < 6; i++) {

		vertices_->push_back(vec3(ctheta * std::cos(phi), ctheta * std::sin(phi), 0)); 

		phi += 2.0 * M_PI / 5.0;
	}

	faces_->push_back(Face(0, 4, 5, true, true, true,false));
	faces_->push_back(Face(0, 3, 4, true, true, true,false));
	faces_->push_back(Face(0, 2, 3, true, true, true,false));
	faces_->push_back(Face(0, 1, 2, true, true, true,false));
	faces_->push_back(Face(0, 5, 1, true, true, true,false));

	CreateAndSetOutportData();

	outportpoint_.setData(vertices_);
	outportface_.setData(faces_);
	
}

Disc::~Disc() {}

void Disc::process() {

	CreateAndSetOutportData();
}



void Disc::onButtonPress() {

	//subdivision
	std::vector<Face> tmp(*faces_);
	faces_->clear();
	size_t initSize = vertices_->size();
	while (!tmp.empty()) {
		subdivide(tmp.back(), initSize);
		tmp.pop_back();
	}

	CreateAndSetOutportData();
	
}

void Disc::onButtonPressback() {

	if (faces_->size() != 5) {
		//subdivisionback
		std::vector<Face> tmp1(*faces_);
		
		//size_t initSize = vertices_->size();
		if (faces_->size() == 20) {
			faces_->clear();
			
			faces_->push_back(Face(0, 4, 5, true, true, true,false));
			faces_->push_back(Face(0, 3, 4, true, true, true,false));
			faces_->push_back(Face(0, 2, 3, true, true, true,false));
			faces_->push_back(Face(0, 1, 2, true, true, true,false));
			faces_->push_back(Face(0, 5, 1, true, true, true,false));
		}
		else {
			faces_->clear();
			for (auto it = tmp1.begin(); it != tmp1.end(); it++) {
				if (it->is_parent_) {
					subdividebackward(*it, tmp1);
				}
			}
		}

		std::vector<std::pair<size_t, bool>> unused_indices;

		size_t index = 0;
		for (auto v : *vertices_) {
			unused_indices.push_back(std::make_pair(index, false));
			index++;
		}

		for (auto f : *faces_) {
			unused_indices.at(f.p1).second = true;
			unused_indices.at(f.p2).second = true;
			unused_indices.at(f.p3).second = true;
		}

		size_t offset = 0;
		for (auto p : unused_indices) {
			if (p.second == false) {
				vertices_->erase((vertices_->begin() + (p.first - offset)));
				offset++;
			}
		}

		CreateAndSetOutportData();
	}

}

void Disc::CreateAndSetOutportData() {

	mesh_ = new BasicMesh();

	//glm::mat4 transform = glm::mat4(1);

	//mesh_->setModelMatrix(glm::translate(mesh_->getModelMatrix(), center_.get()));

	/*transform = glm::translate(transform, vec3(center_.get()));
	
	transform = glm::scale(transform, vec3(radius_.get()));
	
	mesh_->setModelMatrix(transform);*/

	mesh_->setModelMatrix(glm::scale(glm::mat4(1), vec3(radius_.get())));

	auto indexBuffer = mesh_->addIndexBuffer(DrawType::TRIANGLES, ConnectivityType::NONE);

	for (size_t i = 0; i < vertices_->size(); i++) {

		mesh_->addVertex(vertices_->at(i), glm::normalize(vertices_->at(i)), vertices_->at(i), vec4(1.0f, 0.0f, 0.0f, 1.0f));
	}

	//addIndices
	for (auto f : *faces_) {
		
		indexBuffer->add(f.p1);
		indexBuffer->add(f.p2);
		indexBuffer->add(f.p3);
	}

	outportmesh_.setData(mesh_);
	
}



void Disc::subdivide(Face &f, size_t initSize){

	vec3 p1 = vertices_->at(f.p1);
	vec3 p2 = vertices_->at(f.p2);
	vec3 p3 = vertices_->at(f.p3);

	//midpoints
	vec3 p12 = (p2 + p1) / 2.f;
	vec3 p23 = (p3 + p2) / 2.f;
	vec3 p31 = (p1 + p3) / 2.f;

	// lift midpoints on the sphere
	//p12 = glm::normalize(p12);
	if (f.is_on_border_) {
		p23 = glm::normalize(p23);
	}
	//p31 = glm::normalize(p31);

	bool store_p12 = true;
	bool store_p23 = true;
	bool store_p31 = true;
	size_t idx_p12;
	size_t idx_p23;
	size_t idx_p31;


	for (auto i = vertices_->begin() + initSize; i != vertices_->end(); i++) {

		if (*i == p12) {
			store_p12 = false;
			idx_p12 = initSize;
		}
		if (*i == p23) {
			store_p23 = false;
			idx_p23 = initSize;
		}
		if (*i == p31) {
			store_p31 = false;
			idx_p31 = initSize;
		}

		initSize++;
	}

	// add p12, p23 and p31 to vertices
	if (store_p12) {
		vertices_->push_back(p12);
		idx_p12 = vertices_->size() - 1;
	}

	if (store_p23) {
		vertices_->push_back(p23);
		idx_p23 = vertices_->size() - 1;
	}

	if (store_p31) {
		vertices_->push_back(p31);
		idx_p31 = vertices_->size() - 1;
	}

	faces_->push_back(Face(idx_p12, f.p2, idx_p23, f.is_on_border_, false, false,false));
	faces_->push_back(Face(idx_p31, idx_p23, f.p3, f.is_on_border_, false, false,false));
	faces_->push_back(Face(f.p1, idx_p12, idx_p31, false, false, false,false));
	faces_->push_back(Face(idx_p12, idx_p23, idx_p31, false, true, f.is_core_,false));
}

bool Disc::findMatch(size_t index, Face &f) {
	return (index == f.p1 || index == f.p2 || index == f.p3);
}

void Disc::subdividebackward(Face &f, std::vector<Face> faces) {

	std::vector<Face> tmpback;

	for (auto i = faces.begin(); i != faces.end(); i++) { // for all faces
		if ((i->p1 == f.p1 || i->p1 == f.p2 || i->p1 == f.p3) +
			(i->p2 == f.p1 || i->p2 == f.p2 || i->p2 == f.p3) +
			(i->p3 == f.p1 || i->p3 == f.p2 || i->p3 == f.p3) == 2) {
			tmpback.push_back(*i);
		}
		
   }

	size_t index_of_first_vertex_1;
	size_t index_of_first_vertex_2;
	size_t index_of_first_vertex_3;

	size_t shared_vertex_with_next_face_ccw_1;
	
	Face newface;

	/*if (tmpback.at(0) == f) {
		tmpback.erase(tmpback.begin());
	}
	if (tmpback.at(1) == f) {
		tmpback.erase(tmpback.begin() + 1);
	}
	if (tmpback.at(2) == f) {
		tmpback.erase(tmpback.begin() + 2);
	}
	if (tmpback.at(3) == f) {
		tmpback.erase(tmpback.begin() + 3);
	}*/

	if (tmpback.size() == 3) {
		while (!tmpback.empty()) { //tmpback have three faces around f 

			auto j = tmpback.back();
			tmpback.pop_back();

			auto k = tmpback.back();
			tmpback.pop_back();

			auto l = tmpback.back();
			tmpback.pop_back();

			if (!findMatch(j.p1, f)) {
				index_of_first_vertex_1 = j.p1;
				shared_vertex_with_next_face_ccw_1 = j.p2;
			}
			else if (!findMatch(j.p2, f)) {
				index_of_first_vertex_1 = j.p2;
				shared_vertex_with_next_face_ccw_1 = j.p3;
			}
			else if (!findMatch(j.p3, f)) {
				index_of_first_vertex_1 = j.p3;
				shared_vertex_with_next_face_ccw_1 = j.p1;
			}

			if (k.p1 == shared_vertex_with_next_face_ccw_1 || 
				k.p2 == shared_vertex_with_next_face_ccw_1 || 
				k.p3 == shared_vertex_with_next_face_ccw_1) { //k contains shared_vertex_with_next_face_ccw_j

				//find the index of the vertex that is not shared with f

				if (!findMatch(k.p1, f)) {
					index_of_first_vertex_2 = k.p1;
				}
				else if (!findMatch(k.p2, f)) {
					index_of_first_vertex_2 = k.p2;
				}
				else if (!findMatch(k.p3, f)) {
					index_of_first_vertex_2 = k.p3;
				}

				if (!findMatch(l.p1, f)) {
					index_of_first_vertex_3 = l.p1;
				}
				else if (!findMatch(l.p2, f)) {
					index_of_first_vertex_3 = l.p2;
				}
				else if (!findMatch(l.p3, f)) {
					index_of_first_vertex_3 = l.p3;
				}
			}
			else {
				//find the index of the vertex that is not shared with f

				if (!findMatch(l.p1, f)) {
					index_of_first_vertex_2 = l.p1;
				}
				else if (!findMatch(l.p2, f)) {
					index_of_first_vertex_2 = l.p2;
				}
				else if (!findMatch(l.p3, f)) {
					index_of_first_vertex_2 = l.p3;
				}

				if (!findMatch(k.p1, f)) {
					index_of_first_vertex_3 = k.p1;
				}
				else if (!findMatch(k.p2, f)) {
					index_of_first_vertex_3 = k.p2;
				}
				else if (!findMatch(k.p3, f)) {
					index_of_first_vertex_3 = k.p3;
				}
			}

			auto is_border = j.is_on_border_ + k.is_on_border_ + l.is_on_border_;

			newface = Face(index_of_first_vertex_1, index_of_first_vertex_2, index_of_first_vertex_3, is_border == 2, true, f.is_core_,false);

			//tmpback.push_back(f);
			//tmpback.push_back(j);
			//tmpback.push_back(k);
			//tmpback.push_back(l);
	
		}

		/*faces.erase(std::remove_if(faces.begin(), faces.end(),
			[&](face &face) {
			for (auto it = tmpback.begin(); it != tmpback.end(); it++) {
				if (face == *it) {
					return true;
				}
			}
			return false;
		}), faces.end());*/

		faces_->push_back(newface);
	}
}

//subdivision on the boundary only
/*void Disc::subdivide(face &f, size_t initSize){

vec3 p2 = vertices_->at(f.p2);
vec3 p3 = vertices_->at(f.p3);

//midpoints
vec3 p23 = (p3 + p2) / 2.f;

p23 = glm::normalize(p23);


bool store_p23 = true;
size_t idx_p23;


if (store_p23) {
vertices_->push_back(p23);
idx_p23 = vertices_->size() - 1;
}

faces_.push_back(face(f.p1, f.p2, idx_p23));
faces_.push_back(face(f.p1, idx_p23, f.p3));


}*/

} // namespace
