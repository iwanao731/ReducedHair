#pragma once
#include "ofxHEMeshNode.h"


class ofxHEMesh;

struct ofxHEMeshFaceIterator {
		
	ofxHEMeshFaceIterator();
	ofxHEMeshFaceIterator(const ofxHEMesh* hemesh);
	ofxHEMeshFaceIterator(const ofxHEMesh* hemesh, ofxHEMeshFace f);
	ofxHEMeshFaceIterator(const ofxHEMeshFaceIterator& src);
	
	bool operator==(const ofxHEMeshFaceIterator& right);
	bool operator!=(const ofxHEMeshFaceIterator& right);
	
	ofxHEMeshFace& operator*();
	ofxHEMeshFace* operator->();
	
	ofxHEMeshFaceIterator& operator++();
	ofxHEMeshFaceIterator operator++(int);
	ofxHEMeshFaceIterator& operator--();
	ofxHEMeshFaceIterator operator--(int);
	
	const ofxHEMesh* hemesh;
	ofxHEMeshFace f;
};

struct ofxHEMeshEdgeIterator {
		
	ofxHEMeshEdgeIterator();
	ofxHEMeshEdgeIterator(const ofxHEMesh* hemesh);
	ofxHEMeshEdgeIterator(const ofxHEMesh* hemesh, ofxHEMeshHalfedge h);
	ofxHEMeshEdgeIterator(const ofxHEMeshEdgeIterator& src);
	
	bool operator==(const ofxHEMeshEdgeIterator& right);
	bool operator!=(const ofxHEMeshEdgeIterator& right);
	
	ofxHEMeshHalfedge& operator*();
	ofxHEMeshHalfedge* operator->();
	
	ofxHEMeshEdgeIterator& operator++();
	ofxHEMeshEdgeIterator operator++(int);
	ofxHEMeshEdgeIterator& operator--();
	ofxHEMeshEdgeIterator operator--(int);
	
	const ofxHEMesh* hemesh;
	ofxHEMeshHalfedge h;
};

struct ofxHEMeshVertexIterator {
		
	ofxHEMeshVertexIterator();
	ofxHEMeshVertexIterator(const ofxHEMesh* hemesh);
	ofxHEMeshVertexIterator(const ofxHEMesh* hemesh, ofxHEMeshVertex v);
	ofxHEMeshVertexIterator(const ofxHEMeshVertexIterator& src);
	
	bool operator==(const ofxHEMeshVertexIterator& right);
	bool operator!=(const ofxHEMeshVertexIterator& right);
	
	ofxHEMeshVertex& operator*();
	ofxHEMeshVertex* operator->();
	
	ofxHEMeshVertexIterator& operator++();
	ofxHEMeshVertexIterator operator++(int);
	ofxHEMeshVertexIterator& operator--();
	ofxHEMeshVertexIterator operator--(int);
	
	const ofxHEMesh* hemesh;
	ofxHEMeshVertex v;
};

struct ofxHEMeshFaceCirculator {
		
	ofxHEMeshFaceCirculator();
	ofxHEMeshFaceCirculator(const ofxHEMesh* hemesh);
	ofxHEMeshFaceCirculator(const ofxHEMesh* hemesh, ofxHEMeshHalfedge h);
	ofxHEMeshFaceCirculator(const ofxHEMeshFaceCirculator& src);
	
	bool operator==(const ofxHEMeshFaceCirculator& right);
	bool operator!=(const ofxHEMeshFaceCirculator& right);
	
	ofxHEMeshHalfedge& operator*();
	ofxHEMeshHalfedge* operator->();
	
	ofxHEMeshFaceCirculator& operator++();
	ofxHEMeshFaceCirculator operator++(int);
	ofxHEMeshFaceCirculator& operator--();
	ofxHEMeshFaceCirculator operator--(int);
	
	const ofxHEMesh* hemesh;
	ofxHEMeshHalfedge h;
};

struct ofxHEMeshVertexCirculator {
		
	ofxHEMeshVertexCirculator();
	ofxHEMeshVertexCirculator(const ofxHEMesh* hemesh);
	ofxHEMeshVertexCirculator(const ofxHEMesh* hemesh, ofxHEMeshHalfedge h);
	ofxHEMeshVertexCirculator(const ofxHEMeshVertexCirculator& src);
	
	bool operator==(const ofxHEMeshVertexCirculator& right);
	bool operator!=(const ofxHEMeshVertexCirculator& right);
	
	ofxHEMeshHalfedge& operator*();
	ofxHEMeshHalfedge* operator->();
	
	ofxHEMeshVertexCirculator& operator++();
	ofxHEMeshVertexCirculator operator++(int);
	ofxHEMeshVertexCirculator& operator--();
	ofxHEMeshVertexCirculator operator--(int);
	
	const ofxHEMesh* hemesh;
	ofxHEMeshHalfedge h;
};


struct ofxHEMeshTriangle{
	ofxHEMeshTriangle()
	: v1(), v2(), v3()
	{}
	
	ofxHEMeshTriangle(const ofxHEMeshTriangle& src)
	: v1(src.v1), v2(src.v2), v3(src.v3)
	{}
	
	//normal(const ofxHEMesh& hemesh) const;

	ofxHEMeshVertex v1;
	ofxHEMeshVertex v2;
	ofxHEMeshVertex v3;
};

struct ofxHEMeshPolygonSplitter {
	ofxHEMeshPolygonSplitter();
	ofxHEMeshPolygonSplitter(const ofxHEMesh* hemesh);
	ofxHEMeshPolygonSplitter(const ofxHEMesh* hemesh, ofxHEMeshHalfedge h);
	ofxHEMeshPolygonSplitter(const ofxHEMeshPolygonSplitter& src);
	
	bool operator==(const ofxHEMeshPolygonSplitter& right);
	bool operator!=(const ofxHEMeshPolygonSplitter& right);
	
	ofxHEMeshTriangle& operator*();
	ofxHEMeshTriangle* operator->();
	
	ofxHEMeshPolygonSplitter& operator++();
	ofxHEMeshPolygonSplitter operator++(int);
	
	const ofxHEMesh* hemesh;
	ofxHEMeshHalfedge h;
	ofxHEMeshTriangle triangle;
	
protected:
	void firstTriangle();
};