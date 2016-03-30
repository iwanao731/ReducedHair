#pragma once
#include "ofxHEMeshNode.h"

struct ofxHEMeshVertexAdjacency {
	ofxHEMeshVertexAdjacency() : he() {}
	
	ofxHEMeshVertexAdjacency(const ofxHEMeshVertexAdjacency& src)
	: he(src.he)
	{}
	
	ofxHEMeshVertexAdjacency& operator=(const ofxHEMeshVertexAdjacency& src) {
		he = src.he;
		return *this;
	}
	
	ofxHEMeshHalfedge he;
};

struct ofxHEMeshHalfedgeAdjacency {
	ofxHEMeshHalfedgeAdjacency() :
		v(), f(), prev(), next()
	{}
	
	ofxHEMeshHalfedgeAdjacency(const ofxHEMeshHalfedgeAdjacency& src)
	: v(src.v), f(src.f), prev(src.prev), next(src.next)
	{}
	
	ofxHEMeshHalfedgeAdjacency& operator=(const ofxHEMeshHalfedgeAdjacency& src) {
		v = src.v;
		f = src.f;
		prev = src.prev;
		next = src.next;
		return *this;
	}
	
	
	ofxHEMeshVertex v;
	ofxHEMeshFace f;
	ofxHEMeshHalfedge prev;
	ofxHEMeshHalfedge next;
};

struct ofxHEMeshFaceAdjacency {
	ofxHEMeshFaceAdjacency() : he() {}
	
	ofxHEMeshFaceAdjacency(const ofxHEMeshFaceAdjacency& src)
	: he(src.he)
	{}
	
	ofxHEMeshFaceAdjacency& operator=(const ofxHEMeshFaceAdjacency& src) {
		he = src.he;
		return *this;
	}
	
	ofxHEMeshHalfedge he;
};