#include "ofxHEMeshIterators.h"
#include "ofxHEMesh.h"

ofxHEMeshFaceIterator::ofxHEMeshFaceIterator()
: hemesh(NULL), f()
{}

ofxHEMeshFaceIterator::ofxHEMeshFaceIterator(const ofxHEMesh* hemesh)
: hemesh(hemesh), f()
{}

ofxHEMeshFaceIterator::ofxHEMeshFaceIterator(const ofxHEMesh* hemesh, ofxHEMeshFace f)
: hemesh(hemesh), f(f)
{}

ofxHEMeshFaceIterator::ofxHEMeshFaceIterator(const ofxHEMeshFaceIterator& src)
: hemesh(src.hemesh), f(src.f)
{}

bool ofxHEMeshFaceIterator::operator==(const ofxHEMeshFaceIterator& right) {
	return hemesh == right.hemesh && f == right.f;
}

bool ofxHEMeshFaceIterator::operator!=(const ofxHEMeshFaceIterator& right) {
	return !((*this) == right);
}

ofxHEMeshFace& ofxHEMeshFaceIterator::operator*() {
	return f;
}

ofxHEMeshFace* ofxHEMeshFaceIterator::operator->() {
	return &f;
}

ofxHEMeshFaceIterator& ofxHEMeshFaceIterator::operator++() {
	int n = hemesh->faceAdjacency->size();
	do {
		++f.idx;
		if(hemesh->faceHalfedge(f).isValid()) {
			break;
		}
	} while(f.idx < n);
	return *this;
}

ofxHEMeshFaceIterator ofxHEMeshFaceIterator::operator++(int) {
	ofxHEMeshFaceIterator it(*this);
	++(*this);
	return it;
}

ofxHEMeshFaceIterator& ofxHEMeshFaceIterator::operator--() {
	do {
		--f.idx;
		if(hemesh->faceHalfedge(f).isValid()) {
			break;
		}
	} while(f.idx > 0);
	return *this;
}

ofxHEMeshFaceIterator ofxHEMeshFaceIterator::operator--(int) {
	ofxHEMeshFaceIterator it(*this);
	--(*this);
	return it;
}


ofxHEMeshEdgeIterator::ofxHEMeshEdgeIterator()
: hemesh(NULL), h()
{}

ofxHEMeshEdgeIterator::ofxHEMeshEdgeIterator(const ofxHEMesh* hemesh)
: hemesh(hemesh), h()
{}

ofxHEMeshEdgeIterator::ofxHEMeshEdgeIterator(const ofxHEMesh* hemesh, ofxHEMeshHalfedge h)
: hemesh(hemesh), h(h)
{}

ofxHEMeshEdgeIterator::ofxHEMeshEdgeIterator(const ofxHEMeshEdgeIterator& src)
: hemesh(src.hemesh), h(src.h)
{}

bool ofxHEMeshEdgeIterator::operator==(const ofxHEMeshEdgeIterator& right) {
	return hemesh == right.hemesh && h == right.h;
}

bool ofxHEMeshEdgeIterator::operator!=(const ofxHEMeshEdgeIterator& right) {
	return !((*this) == right);
}

ofxHEMeshHalfedge& ofxHEMeshEdgeIterator::operator*() {
	return h;
}

ofxHEMeshHalfedge* ofxHEMeshEdgeIterator::operator->() {
	return &h;
}

ofxHEMeshEdgeIterator& ofxHEMeshEdgeIterator::operator++() {
	int n = hemesh->halfedgeAdjacency->size();
	do {
		h.idx += 2;
		if(hemesh->halfedgeVertex(h).isValid()) {
			break;
		}
	} while(h.idx < n);
	h.idx = MIN(h.idx, n);
	return *this;
}

ofxHEMeshEdgeIterator ofxHEMeshEdgeIterator::operator++(int) {
	ofxHEMeshEdgeIterator it(*this);
	++(*this);
	return it;
}

ofxHEMeshEdgeIterator& ofxHEMeshEdgeIterator::operator--() {
	do {
		h.idx -= 2;
		if(hemesh->halfedgeVertex(h).isValid()) {
			break;
		}
	} while(h.idx > 0);
	h.idx = MAX(h.idx, 0);
	return *this;
}

ofxHEMeshEdgeIterator ofxHEMeshEdgeIterator::operator--(int) {
	ofxHEMeshEdgeIterator it(*this);
	--(*this);
	return it;
}

ofxHEMeshVertexIterator::ofxHEMeshVertexIterator()
: hemesh(NULL), v()
{}

ofxHEMeshVertexIterator::ofxHEMeshVertexIterator(const ofxHEMesh* hemesh)
: hemesh(hemesh), v()
{}

ofxHEMeshVertexIterator::ofxHEMeshVertexIterator(const ofxHEMesh* hemesh, ofxHEMeshVertex v)
: hemesh(hemesh), v(v)
{}

ofxHEMeshVertexIterator::ofxHEMeshVertexIterator(const ofxHEMeshVertexIterator& src)
: hemesh(src.hemesh), v(src.v)
{}

bool ofxHEMeshVertexIterator::operator==(const ofxHEMeshVertexIterator& right) {
	return hemesh == right.hemesh && v == right.v;
}

bool ofxHEMeshVertexIterator::operator!=(const ofxHEMeshVertexIterator& right) {
	return !((*this) == right);
}

ofxHEMeshVertex& ofxHEMeshVertexIterator::operator*() {
	return v;
}

ofxHEMeshVertex* ofxHEMeshVertexIterator::operator->() {
	return &v;
}

ofxHEMeshVertexIterator& ofxHEMeshVertexIterator::operator++() {
	int n = hemesh->vertexAdjacency->size();
	do {
		++v.idx;
		if(hemesh->vertexHalfedge(v).isValid()) {
			break;
		}
	} while(v.idx < n);
	return *this;
}

ofxHEMeshVertexIterator ofxHEMeshVertexIterator::operator++(int) {
	ofxHEMeshVertexIterator it(*this);
	++(*this);
	return it;
}

ofxHEMeshVertexIterator& ofxHEMeshVertexIterator::operator--() {
	do {
		--v.idx;
		if(hemesh->vertexHalfedge(v).isValid()) {
			break;
		}
	} while(v.idx > 0);
	return *this;
}

ofxHEMeshVertexIterator ofxHEMeshVertexIterator::operator--(int) {
	ofxHEMeshVertexIterator it(*this);
	--(*this);
	return it;
}


ofxHEMeshFaceCirculator::ofxHEMeshFaceCirculator()
: hemesh(NULL), h()
{}

ofxHEMeshFaceCirculator::ofxHEMeshFaceCirculator(const ofxHEMesh* hemesh)
: hemesh(hemesh), h()
{}

ofxHEMeshFaceCirculator::ofxHEMeshFaceCirculator(const ofxHEMesh* hemesh, ofxHEMeshHalfedge h)
: hemesh(hemesh), h(h)
{}

ofxHEMeshFaceCirculator::ofxHEMeshFaceCirculator(const ofxHEMeshFaceCirculator& src)
: hemesh(src.hemesh), h(src.h)
{}

bool ofxHEMeshFaceCirculator::operator==(const ofxHEMeshFaceCirculator& right) {
	return hemesh == right.hemesh && h == right.h;
}
bool ofxHEMeshFaceCirculator::operator!=(const ofxHEMeshFaceCirculator& right) {
	return !(*this == right);
}

ofxHEMeshHalfedge& ofxHEMeshFaceCirculator::operator*() {
	return h;
}

ofxHEMeshHalfedge* ofxHEMeshFaceCirculator::operator->() {
	return &h;
}

ofxHEMeshFaceCirculator& ofxHEMeshFaceCirculator::operator++() {
	h = hemesh->halfedgeNext(h);
	return *this;
}

ofxHEMeshFaceCirculator ofxHEMeshFaceCirculator::operator++(int) {
	ofxHEMeshFaceCirculator it(*this);
	++(*this);
	return it;
}

ofxHEMeshFaceCirculator& ofxHEMeshFaceCirculator::operator--() {
	h = hemesh->halfedgePrev(h);
	return *this;
}

ofxHEMeshFaceCirculator ofxHEMeshFaceCirculator::operator--(int) {
	ofxHEMeshFaceCirculator it(*this);
	--(*this);
	return it;
}



ofxHEMeshVertexCirculator::ofxHEMeshVertexCirculator()
: hemesh(NULL), h()
{}

ofxHEMeshVertexCirculator::ofxHEMeshVertexCirculator(const ofxHEMesh* hemesh)
: hemesh(hemesh), h()
{}

ofxHEMeshVertexCirculator::ofxHEMeshVertexCirculator(const ofxHEMesh* hemesh, ofxHEMeshHalfedge h)
: hemesh(hemesh), h(h)
{}

ofxHEMeshVertexCirculator::ofxHEMeshVertexCirculator(const ofxHEMeshVertexCirculator& src)
: hemesh(src.hemesh), h(src.h)
{}

bool ofxHEMeshVertexCirculator::operator==(const ofxHEMeshVertexCirculator& right) {
	return hemesh == right.hemesh && h == right.h;
}

bool ofxHEMeshVertexCirculator::operator!=(const ofxHEMeshVertexCirculator& right) {
	return !(*this == right);
}

ofxHEMeshHalfedge& ofxHEMeshVertexCirculator::operator*() {
	return h;
}

ofxHEMeshHalfedge* ofxHEMeshVertexCirculator::operator->() {
	return &h;
}

ofxHEMeshVertexCirculator& ofxHEMeshVertexCirculator::operator++() {
	h = hemesh->halfedgeSinkCCW(h);
	return *this;
}

ofxHEMeshVertexCirculator ofxHEMeshVertexCirculator::operator++(int) {
	ofxHEMeshVertexCirculator it(*this);
	++(*this);
	return it;
}

ofxHEMeshVertexCirculator& ofxHEMeshVertexCirculator::operator--() {
	h = hemesh->halfedgeSinkCW(h);
	return *this;
}

ofxHEMeshVertexCirculator ofxHEMeshVertexCirculator::operator--(int) {
	ofxHEMeshVertexCirculator it(*this);
	--(*this);
	return it;
}


ofxHEMeshPolygonSplitter::ofxHEMeshPolygonSplitter()
: hemesh(NULL), h()
{}

ofxHEMeshPolygonSplitter::ofxHEMeshPolygonSplitter(const ofxHEMesh* hemesh)
: hemesh(hemesh), h()
{}


ofxHEMeshPolygonSplitter::ofxHEMeshPolygonSplitter(const ofxHEMesh* hemesh, ofxHEMeshHalfedge h)
: hemesh(hemesh), h(h)
{}

ofxHEMeshPolygonSplitter::ofxHEMeshPolygonSplitter(const ofxHEMeshPolygonSplitter& src)
: hemesh(src.hemesh), h(src.h), triangle(src.triangle)
{}

bool ofxHEMeshPolygonSplitter::operator==(const ofxHEMeshPolygonSplitter& right) {
	return hemesh == right.hemesh && h == right.h;
}

bool ofxHEMeshPolygonSplitter::operator!=(const ofxHEMeshPolygonSplitter& right) {
	return !(*this == right);
}

ofxHEMeshTriangle& ofxHEMeshPolygonSplitter::operator*() {
	firstTriangle();
	return triangle;
}

ofxHEMeshTriangle* ofxHEMeshPolygonSplitter::operator->() {
	firstTriangle();
	return &triangle;
}

ofxHEMeshPolygonSplitter& ofxHEMeshPolygonSplitter::operator++() {
	firstTriangle();
	h = hemesh->halfedgeNext(h);
	triangle.v2 = triangle.v3;
	triangle.v3 = hemesh->halfedgeVertex(h);
	return *this;
}

ofxHEMeshPolygonSplitter ofxHEMeshPolygonSplitter::operator++(int) {
	ofxHEMeshPolygonSplitter it(*this);
	++(*this);
	return it;
}

void ofxHEMeshPolygonSplitter::firstTriangle() {
	if(!triangle.v1.isValid()) {
		triangle.v1 = hemesh->halfedgeVertex(h);
		h = hemesh->halfedgeNext(h);
		triangle.v2 = hemesh->halfedgeVertex(h);
		h = hemesh->halfedgeNext(h);
		triangle.v3 = hemesh->halfedgeVertex(h);
	}
}