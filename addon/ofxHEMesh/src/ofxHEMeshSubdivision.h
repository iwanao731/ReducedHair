#pragma once
#include "ofxHEMesh.h"
#include <set>

using std::set;

class ofxHEMeshCornerCutSubdivision{
public:
	typedef map<ofxHEMeshVertex, ofxHEMeshVertex> VertexMap;
	typedef map<ofxHEMeshFace, VertexMap> CornerMap;

	ofxHEMeshCornerCutSubdivision(ofxHEMesh& hemesh);
	virtual ~ofxHEMeshCornerCutSubdivision();
	
	void apply();
	
protected:
	virtual void vertexWeights(vector<ofxHEMesh::Scalar>& weights, ofxHEMeshFace f) = 0;

	void processFaces();
	void createVertexFaces();
	void createHalfedgeFaces();
	void createNewVertices();
	void createNewFaces();


	ofxHEMesh& hemesh;
	vector<ofxHEMesh::ExplicitFace> faces;
	CornerMap cornerVertices;
	vector<ofxHEMesh::Point> cornerPoints;
	set<ofxHEMeshVertex> boundaryVertices;
};


class ofxHEMeshDooSabinSubdivision : public ofxHEMeshCornerCutSubdivision{
public:
	ofxHEMeshDooSabinSubdivision(ofxHEMesh& hemesh);
	
protected:
	void vertexWeights(vector<ofxHEMesh::Scalar>& weights, ofxHEMeshFace f);
};


// see: http://www.viz.tamu.edu/faculty/ergun/research/topology/papers/ijsm01.pdf
// generally tension should be in the range [5/12, 1]
class ofxHEMeshModifiedCornerCutSubdivision : public ofxHEMeshCornerCutSubdivision{
public:
	ofxHEMeshModifiedCornerCutSubdivision(ofxHEMesh& hemesh, ofxHEMesh::Scalar tension);
	
protected:
	void vertexWeights(vector<ofxHEMesh::Scalar>& weights, ofxHEMeshFace f);
	
	ofxHEMesh::Scalar tension;
};


class ofxHEMeshFacePeel : public ofxHEMeshModifiedCornerCutSubdivision {
public:
	ofxHEMeshFacePeel(ofxHEMesh& hemesh, ofxHEMesh::Scalar thickness);

	void apply();

protected:
	void subdivide();
	void createCrust();

	ofxHEMesh innerHemesh;
	int numFaces;
};