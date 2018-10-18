

#include "Sphere.h"
#include <inviwo/core/common/inviwo.h>
#include <bitset>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/opengl/image/layergl.h>
#include <inviwo/core/datastructures/image/layerram.h>


namespace inviwo {

ProcessorClassIdentifier(Sphere, "org.inviwo.Sphere")
ProcessorDisplayName(Sphere, "Sphere")
ProcessorTags(Sphere, Tags::None);
ProcessorCategory(Sphere, "Sphere Visualization");
ProcessorCodeState(Sphere, CodeState::Experimental);

Sphere::Sphere()
	: Processor(),
	 outportmesh_("outportmesh_")
	, subdivide_btn_("subdivide_btn", "Subdivide")
	, subdividebackward_btn_("subdividebackward_btn", "Subdivide backward")
	, outportpoint_("vec3_outport")
	, outportface_("face_outport")
	, radius_("sphereRadius", "Radius", 0.5f, 0.01f, 0.5f, .01f)
	, vertices_(new std::vector<vec3>())
	, faces_(new std::vector<Face>())
	
{

	//vertices_ = new vec3[12];
	addPort(outportmesh_);
	addPort(outportpoint_);
	addPort(outportface_);

	addProperty(subdivide_btn_);
	addProperty(subdividebackward_btn_);
	addProperty(radius_);

	subdivide_btn_.onChange([&]() { onButtonPress(); });

	subdividebackward_btn_.onChange([&]() { onButtonPressback(); });

	model_matrix_ = glm::mat4(1);
	

	double theta = 26.56505117707799 * M_PI / 180.0; // refer paper for theta value
	double stheta = std::sin(theta);
	double ctheta = std::cos(theta);

	vertices_->push_back(vec3(0.0f, 0.0f, -1.0f)); // the lower vertex

	// the lower pentagon
	double phi = M_PI / 5.0;
	for (int i = 1; i < 6; ++i) {

		vertices_->push_back(vec3(ctheta * std::cos(phi), ctheta * std::sin(phi), -stheta));

		phi += 2.0 * M_PI / 5.0;
	}

	// the upper pentagon
	phi = 0.0;
	for (int i = 6; i < 11; ++i) {

		vertices_->push_back(vec3(ctheta * std::cos(phi), ctheta * std::sin(phi), stheta));

		phi += 2.0 * M_PI / 5.0;
	}


	vertices_->push_back(vec3(0.0f, 0.0f, 1.0f)); // the upper vertex

	faces_->push_back(Face(0, 2, 1, false,true, true,false));
	faces_->push_back(Face(0, 3, 2, false, true, true, false));
	faces_->push_back(Face(0, 4, 3, false,true, true, false));
	faces_->push_back(Face(0, 5, 4, false,true, true, false));
	faces_->push_back(Face(0, 1, 5, false,true, true, false));

	faces_->push_back(Face(1, 2, 7, false,true, true, false));
	faces_->push_back(Face(2, 3, 8, false,true, true, false));
	faces_->push_back(Face(3, 4, 9, false,true, true, false));
	faces_->push_back(Face(4, 5, 10,false,true, true, false));
	faces_->push_back(Face(5, 1, 6, false,true, true, false));

	faces_->push_back(Face(1, 7, 6, false,true, true, false));
	faces_->push_back(Face(2, 8, 7, false,true, true, false));
	faces_->push_back(Face(3, 9, 8, false,true, true, false));
	faces_->push_back(Face(4, 10, 9,false,true, true, false));
	faces_->push_back(Face(5, 6, 10,false,true, true, false));

	faces_->push_back(Face(6, 7, 11, false,true, true, false));
	faces_->push_back(Face(7, 8, 11, false,true, true, false));
	faces_->push_back(Face(8, 9, 11, false,true, true, false));
	faces_->push_back(Face(9, 10, 11,false,true, true, false));
	faces_->push_back(Face(10, 6, 11,false,true, true, false));
	
	CreateAndSetOutportData();


	outportpoint_.setData(vertices_);
    outportface_.setData(faces_);
	
	
}

Sphere::~Sphere() {}

void Sphere::process() {
	CreateAndSetOutportData();

	//distance of two vertex
	auto oneface = faces_->back();

	vec3 oneface_p1 = vertices_->at(oneface.p1);
	vec3 oneface_p2 = vertices_->at(oneface.p2);
	vec3 oneface_p3 = vertices_->at(oneface.p3);

	auto dis_vertex = glm::distance(oneface_p1, oneface_p2);

	LogInfo("distance of two vertex = " << dis_vertex);

}

void Sphere::onButtonPress() {

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

void Sphere::onButtonPressback() {

	if ((faces_->size() != 20)) {
		
		std::vector<Face> tmp1(*faces_);
	
		if (faces_->size() == 80) {
			faces_->clear();
			candidatesParent_.clear();
			//candidatesChild_.clear();

			faces_->push_back(Face(0, 2, 1, false,true, true, false));
			faces_->push_back(Face(0, 3, 2, false,true, true, false));
			faces_->push_back(Face(0, 4, 3, false,true, true, false));
			faces_->push_back(Face(0, 5, 4, false,true, true, false));
			faces_->push_back(Face(0, 1, 5, false,true, true, false));

			faces_->push_back(Face(1, 2, 7, false,true, true, false));
			faces_->push_back(Face(2, 3, 8, false,true, true, false));
			faces_->push_back(Face(3, 4, 9, false,true, true, false));
			faces_->push_back(Face(4, 5, 10,false,true, true, false));
			faces_->push_back(Face(5, 1, 6, false,true, true, false));

			faces_->push_back(Face(1, 7, 6, false,true, true, false));
			faces_->push_back(Face(2, 8, 7, false,true, true, false));
			faces_->push_back(Face(3, 9, 8, false,true, true, false));
			faces_->push_back(Face(4, 10, 9,false,true, true, false));
			faces_->push_back(Face(5, 6, 10,false,true, true, false));

			faces_->push_back(Face(6, 7, 11, false,true, true, false));
			faces_->push_back(Face(7, 8, 11, false,true, true, false));
			faces_->push_back(Face(8, 9, 11, false,true, true, false));
			faces_->push_back(Face(9, 10, 11,false,true, true, false));
			faces_->push_back(Face(10, 6, 11,false,true, true, false));

			}

		else {
		
		faces_->clear();
		/*for (auto it = tmp1.begin(); it != tmp1.end(); it+=4) {

			if (it->is_back_){

			it->is_parent_ = true;
			it->is_back_ = false;
			it->is_core_ = true;

			(it + 1)->is_parent_ = false;
			(it + 1)->is_back_ = false;
			(it + 1)->is_core_ = false;

			(it + 2)->is_parent_ = false;
			(it + 2)->is_back_ = false;
			(it + 2)->is_core_ = false;

			(it + 3)->is_parent_ = false;
			(it + 3)->is_back_ = false;
			(it + 3)->is_core_ = false;

			}
			}*/

		/*std::vector<face> tmpback1;
		std::vector<face> tmp2;
	
		for (auto it = tmp1.begin(); it != tmp1.end(); it++) {
		
		std::vector<face> parent;
		unsigned int offset = 0;

		if (tmp1.back().is_back_) {

			while (!tmp1.empty()) {

				candidates.clear();
				candidates.push_back(tmp1[tmp1.size() - 1 - offset]);

						for (auto i = tmp1.begin(); i != tmp1.end(); i++) { // for all faces
							if ((i->p1 == tmp1[tmp1.size() - 1 - offset].p1 || i->p1 == tmp1[tmp1.size() - 1 - offset].p2 || i->p1 == tmp1[tmp1.size() - 1 - offset].p3) +
								(i->p2 == tmp1[tmp1.size() - 1 - offset].p1 || i->p2 == tmp1[tmp1.size() - 1 - offset].p2 || i->p2 == tmp1[tmp1.size() - 1 - offset].p3) +
								(i->p3 == tmp1[tmp1.size() - 1 - offset].p1 || i->p3 == tmp1[tmp1.size() - 1 - offset].p2 || i->p3 == tmp1[tmp1.size() - 1 - offset].p3) == 2) {
								   candidates.push_back(*i);
							}
						}	
					}
					
				  }
					else {
						continue;
					
					}

					if (candidates.size() != 4) {
						if (offset <= tmp1.size()){
							offset++;
							continue;}
						else
							return;
					}
					else {
						offset = 0;
						tmp1.erase(std::remove_if(tmp1.begin(), tmp1.end(), [&](face &k) {
							for (auto j = candidates.begin(); j != candidates.end(); j++) {
								if ((k.p1 == j->p1 || k.p1 == j->p2 || k.p1 == j->p3) +
									(k.p2 == j->p1 || k.p2 == j->p2 || k.p2 == j->p3) +
									(k.p3 == j->p1 || k.p3 == j->p2 || k.p3 == j->p3) == 3){
									return true;
								}
							}
							return false;
						}), tmp1.end());

						tmpback1.insert(tmpback1.end(), candidates.begin(), candidates.end());

						while (!tmpback1.empty()) {

							auto s1 = tmpback1.back();
							tmpback1.pop_back();
							s1.is_parent_ = false;
							s1.is_back_ = false;
							s1.is_core_ = false;
							tmp2.push_back(s1);

							auto s2 = tmpback1.back();
							tmpback1.pop_back();
							s2.is_parent_ = false;
							s2.is_back_ = false;
							s2.is_core_ = false;
							tmp2.push_back(s2);

							auto s3 = tmpback1.back();
							tmpback1.pop_back();
							s3.is_parent_ = false;
							s3.is_back_ = false;
							s3.is_core_ = false;
							tmp2.push_back(s3);

							auto s4 = tmpback1.back();//parent
							tmpback1.pop_back();
							s4.is_parent_ = true;
							s4.is_back_ = false;
							s4.is_core_ = true;
							tmp2.push_back(s4);

						}	
					}
				}
			tmp1.insert(tmp1.end(), tmp2.begin(), tmp2.end());
		}*/

        candidatesParent_.clear();
		//candidatesChild_.clear();
		
		for (auto it = tmp1.begin(); it != tmp1.end(); it++) {
			if (it->is_parent_) {
				subdividebackward(*it, tmp1);
				
			}		
		}

		
	}
	   
	    //Erase unused vertices
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

void Sphere::CreateAndSetOutportData() {

	mesh_ = new BasicMesh();

	mesh_->setModelMatrix(glm::scale(glm::mat4(1), vec3(radius_.get())));
	
	auto indexBuffer = mesh_->addIndexBuffer(DrawType::TRIANGLES, ConnectivityType::NONE);

	for (size_t i = 0; i < vertices_->size(); i++) {

		/*
	// find all faces that use p by looping through all faces and check if p1, p2 or p3 == i
		std::vector<face> faces_of_p;
		for (auto f : faces_) {
			if (f.p1 == i || f.p2 == i ||  f.p3 == i) {
			// the vertex belongs to f
				faces_of_p.push_back(f);
			}
		}

		// faces_of_p now contains all faces that use p
		// for all faces in faces_of_p calculate the normal
		int j = 1;
		vec3 sum = vec3(0, 0, 0);
		for (auto fp : faces_of_p) {

			sum += getNormal(fp);
			j++;

		}

		// average the normals
		vec3 average = glm::normalize(sum /(float)j);
		//vec3 new_pos = (mesh_->getModelMatrix() * vec4(vertices_->at(i), 1)).xyz;
		*/

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

void Sphere::subdivide(Face &f, size_t initSize){

	vec3 p1 = vertices_->at(f.p1);
	vec3 p2 = vertices_->at(f.p2);
	vec3 p3 = vertices_->at(f.p3);
	
	//midpoints
	vec3 p12 = (p2 + p1) / 2.f;
	vec3 p23 = (p3 + p2) / 2.f;
	vec3 p31 = (p1 + p3) / 2.f;	

	// lift midpoints on the sphere
	p12 = glm::normalize(p12);
	p23 = glm::normalize(p23);
	p31 = glm::normalize(p31);

	bool store_p12 = true;
	bool store_p23 = true;
	bool store_p31 = true;
	size_t idx_p12;
	size_t idx_p23;
	size_t idx_p31;


	for (auto i = vertices_->begin() + initSize; i != vertices_->end(); i++) {

		if (*i == p12) {
			store_p12 = false;
			idx_p12 = initSize ;
		}
		if (*i == p23) {
			store_p23 = false;
			idx_p23 = initSize ;
		}
		if (*i == p31) {
			store_p31 = false;
			idx_p31 = initSize ;
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
	
	faces_->push_back(Face(f.p1, idx_p12, idx_p31,false,false, false, true));
	faces_->push_back(Face(f.p2, idx_p23, idx_p12,false,false, false, true));
	faces_->push_back(Face(f.p3, idx_p31, idx_p23,false,false, false, true));
	faces_->push_back(Face(idx_p12, idx_p23, idx_p31,false,true, f.is_core_, false));

	}

bool Sphere::findMatch(size_t index, Face &f) {
	return (index == f.p1 || index == f.p2 || index == f.p3);
}

void Sphere::subdividebackward(Face &f, std::vector<Face> faces) {

	std::vector<Face> tmpback;

	for (auto i = faces.begin(); i != faces.end(); i++) { // for all faces

		if ((i->p1 == f.p1 || i->p1 == f.p2 || i->p1 == f.p3) +
			(i->p2 == f.p1 || i->p2 == f.p2 || i->p2 == f.p3) +
			(i->p3 == f.p1 || i->p3 == f.p2 || i->p3 == f.p3) == 2) {
			tmpback.push_back(*i);//tmpback have three faces around f 
		}
	}

	size_t index_of_first_vertex_1;
	size_t index_of_first_vertex_2;
	size_t index_of_first_vertex_3;

	size_t shared_vertex_with_next_face_ccw_1;

	Face newface;
	newface.is_child_ = false;

	if (tmpback.size() == 3) { //tmpback have three faces around f 
		while (!tmpback.empty()) {

			auto j = tmpback.back();
			tmpback.pop_back();

			auto k = tmpback.back();
			tmpback.pop_back();

			auto l = tmpback.back();
			tmpback.pop_back();

			//find the index of the vertex that is not shared with f in j face
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
				k.p3 == shared_vertex_with_next_face_ccw_1) { //k contains shared_vertex_with_next_face_ccw_1

				//find the index of the vertex that is not shared with f in k face

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
				//find the index of the vertex that is not shared with f in l face

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

			//auto is_border = j.is_on_border_ + k.is_on_border_ + l.is_on_border_;
			if (candidatesParent_.size() == 0){

				newface = Face(index_of_first_vertex_1, index_of_first_vertex_2, index_of_first_vertex_3, false, true, f.is_core_, false);
				candidatesParent_.push_back(newface);
			}
			else{

				for (auto ii = candidatesParent_.begin(); ii != candidatesParent_.end(); ii++) { //all parents

					 if (((ii->p1 == index_of_first_vertex_1 || ii->p1 == index_of_first_vertex_2 || ii->p1 == index_of_first_vertex_3) +
						(ii->p2 == index_of_first_vertex_1 || ii->p2 == index_of_first_vertex_2 || ii->p2 == index_of_first_vertex_3) +
						(ii->p3 == index_of_first_vertex_1 || ii->p3 == index_of_first_vertex_2 || ii->p3 == index_of_first_vertex_3) == 2)) {

						newface = Face(index_of_first_vertex_1, index_of_first_vertex_2, index_of_first_vertex_3, false, false, false, true);//child
						break;
					}
				}
				if (newface.is_child_ != true){

					newface = Face(index_of_first_vertex_1, index_of_first_vertex_2, index_of_first_vertex_3, false, true, f.is_core_, false);//parent
					candidatesParent_.push_back(newface);
				}

				//if (newface.is_child_ == true){//child

				//	//newface = face(index_of_first_vertex_1, index_of_first_vertex_2, index_of_first_vertex_3, true, f.is_core_, false);//child
				//	candidatesChild_.push_back(newface);
				//}

			}
		}
	}

	faces_->push_back(newface);
}

} // namespace
